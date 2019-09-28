#!/usr/bin/env python

import rospy
import yaml
import time
from std_msgs.msg import String, UInt8, Header
from visualization_msgs.msg import Marker, MarkerArray
from cut_mission.msg import Waypoint, WaypointPairLabeled

class MissionPlanner:
    """ @brief executes mission from mission file
        Reads mission from a .yaml mission file (template in /missions),
        Sequentially passes between waypoints, executing behaviors accompanying
        each mission path segment, until reaching an end behavior.
        """

    def __init__(self):

        # Set up ros stuff
        rospy.init_node('mission_planner')
        self.state_pub = rospy.Publisher('/state_controller/cmd_state', String, queue_size=1)
        self.behavior_pub = rospy.Publisher('/mission_planner/out_behavior', WaypointPairLabeled, queue_size=1)
        self.wpvis_pub = rospy.Publisher('/mission_planner/vis_waypoints', MarkerArray, queue_size=1)
        rospy.Subscriber('/mission_planner/in_behavior', String, self.in_behavior_cb)
        self.rate = rospy.Rate(1)
        self.name = 'mp'

        # Set up mission
        time.sleep(0.5)
        #TODO; Better error message.
        self.load_mission(rospy.get_param('/cut_mission/mission_file'))
        self.load_markers()
        print(self.markers)
        #self.wpvis_pub.publish(self.markers)
        self.curr_waypoint = None

        # Get behaviors - start mission if no errors
        self.behaviors = {}
        if not self.update_behaviors():
            time.sleep(0.5)
            self.start_mission()
        else:
            return

    def in_behavior_cb(self, msg):
        """ @brief end segment if reached wp is next in mission
            """
        if msg.data == self.curr_waypoint.waypoint1.behavior:
            self.next_behavior()
        else:
            rospy.logwarn("%s - Recieved incorrect waypoint", self.name)

    def load_mission(self, file):
        """ @brief load waypoints into list of Waypoints
            @param[in] file: .yaml mission file
            @param[out] self.waypoints: list of ROS waypoint msgs
            """

        with open(file, 'r') as f:

            # Load mission yaml file for parsing
            doc = yaml.load(f)
            self.mission = doc['title']
            rospy.loginfo("%s - Loading mission %s", self.name, self.mission)
            self.waypoints = [None] * len(doc['waypoints'])

            # Create ROS Waypoint, add to waypoints attribute
            for i in range(len(doc['waypoints'])):
                msg = Waypoint()
                msg.index = doc['waypoints'][i]['index']
                msg.behavior = doc['waypoints'][i]['behavior']
                msg.forward = bool(doc['waypoints'][i]['forward'])
                msg.autocontinue = bool(doc['waypoints'][i]['autocontinue'])
                msg.point.x = int(doc['waypoints'][i]['point']['x'])
                msg.point.y = int(doc['waypoints'][i]['point']['y'])
                msg.point.z = int(doc['waypoints'][i]['point']['z'])
                self.waypoints[i] = msg
                rospy.loginfo("%s - Loaded waypoint %i", self.name, self.waypoints[i].index)

            return

    def load_markers(self):
        """ @brief init marker attr based on loaded waypoints
            """

        self.markers = MarkerArray()
        for i in range(len(self.waypoints)):
            marker = Marker()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.header = Header()
            marker.header.frame_id = '/odom'
            marker.header.stamp = rospy.get_rostime()
            marker.id = i
            marker.pose.position.x = self.waypoints[i].point.x
            marker.pose.position.y = self.waypoints[i].point.y
            marker.pose.position.z = self.waypoints[i].point.z
            marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5
            marker.color.g = marker.color.b = 0.0
            marker.color.a = marker.color.r = 1.0
            marker.lifetime = rospy.Duration()
            self.markers.markers.append(marker)

    def print_mission(self):
        """ @brief prints mission waypts & segments
            """
        try:
            print("Mission: " + self.mission)
            for i in range(len(self.waypoints) - 1):
                print("| Execute behavior " + self.waypoints[i].behavior + " from " + str(self.waypoints[i].index) + " to " + str(self.waypoints[i+1].index))
            print("| Execute behavior " + self.waypoints[-1].behavior + " at " + str(self.waypoints[-1].index))
        except AttributeError as e:
            print("No mission loaded yet. Error: " + str(e))

    def start_mission(self, init_wp = 0):
        """ @brief run mission from waypoint
            @param[in] init_wp: index of starting waypoint; defaults to 0
            @param[out] curr_wp_index: updates current waypoint index attr
            """
        self.curr_wp_index = init_wp
        rospy.loginfo("%s - Starting mission: %s", self.name, self.mission)
        self.start_behavior(self.curr_wp_index)
        return

    def end_mission(self):
        """ @brief end mission, log necessary information
            """
        self.end_behavior(self.curr_wp_index)
        self.curr_wp_index = None
        rospy.loginfo('Ending mission: ' + self.mission)
        rospy.signal_shutdown('Mission successfully completed')
        return

    def update_behaviors(self):
        ''' @brief Updates behavior parameters
            updateBehaviors checks the behavior namespace on the parameter server and
            populates behavior_list with the behaviors listed there.
            TODO: Ensure no duplicates in behavior vector
            @return [int]: error repr
            '''
        rospy.loginfo("%s - Loading behaviors", self.name)
        if rospy.has_param('/behaviors'):
            temp_list = rospy.get_param('/behaviors')
        else:
            rospy.logerr("%s - no /behaviors/ parameters - did you set up the parameter server?", self.name)
            rospy.signal_shutdown('no /behaviors/ parameters')
            return 1

        if len(temp_list) > 0:
            # Populate behavior list with parameter-defined behaviors
            for behavior in temp_list:
                self.behaviors[behavior] = temp_list[behavior]
                rospy.loginfo("%s - Loaded behavior " + behavior + " with id " + str(temp_list[behavior]), self.name)
        return 0

    def next_behavior(self):
        """ @brief end current behavior, setup & start next behavior
            """
        self.end_behavior(self.curr_wp_index)
        try:
            self.curr_wp_index = self.curr_wp_index + 1
            self.start_behavior(self.curr_wp_index)
            return

        except IndexError as e:
            rospy.loginfo("Reached end of mission, shutting down mission planner.")
            rospy.signal_shutdown()
            return

    def start_behavior(self, wp_index):
        ''' @brief init behavior for waypoint @ index
            @param[in] wp_index: index into waypoints attr for current waypoint
            '''
        msg = WaypointPairLabeled()
        msg.label = self.behaviors[self.waypoints[wp_index].behavior]
        msg.waypoint1 = self.waypoints[wp_index]
        self.markers.markers[wp_index].color.g = 1.0
        self.markers.markers[wp_index].color.r = 0.0
        if not len(self.waypoints) == wp_index + 1:
            msg.waypoint2 = self.waypoints[wp_index + 1]
        else:
            msg.waypoint2 = msg.waypoint1
        rospy.loginfo("%s - starting behavior: %s", self.name, msg.waypoint1.behavior)
        msg_state = String()
        msg_state.data = msg.waypoint1.behavior
        self.state_pub.publish(msg_state)
        self.behavior_pub.publish(msg)
        self.curr_waypoint = msg
        return

    def end_behavior(self, wp_index):
        """ @brief ends execution of waypoint behavior
            @param[in] wp_1: Waypoint to end
            """

        # TODO:run behavior end function
        self.state_pub.publish(self.behaviors['safety'])
        rospy.loginfo("%s - ending behavior: %s", self.name, self.waypoints[wp_index].behavior)
        self.markers.markers[wp_index].color.r = 1.0
        self.markers.markers[wp_index].color.g = 0.0
        if not self.waypoints[wp_index].autocontinue:
            pass
            #TODO Add pause for user input here

        return

    def run(self):
        while not rospy.is_shutdown():
            self.wpvis_pub.publish(self.markers)
            self.rate.sleep()

if __name__ == "__main__":
    m1 = MissionPlanner()
    m1.run()
