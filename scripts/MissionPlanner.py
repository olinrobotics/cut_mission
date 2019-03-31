#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String, Int16, Header
from cut_planner.msg import Waypoint, WaypointPairLabeled

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
        rospy.Subscriber('/mission_planner/in_behavior', Int16, self.in_behavior_cb)

        # Set up mission
        self.load_mission('../config/p2p_test.yaml')
        self.curr_waypoint = None

        # Get behaviors - start mission if no errors
        self.behaviors = {}
        if not self.update_behaviors():
            self.start_mission()
        else:
            return

    def in_behavior_cb(self, msg):
        """ @brief end segment if reached wp is next in mission
            """
        if msg.data == self.curr_wp_index + 1:
            if len(self.waypoints) > msg.data:
                rospy.logdebug("Recieved next waypoint")
                self.next_behavior()
            else:
                rospy.logdebug("Recieved last waypoint")
                self.end_mission()
        else:
            rospy.logwarn("Recieved incorrect waypoint")

    def load_mission(self, file):
        """ @brief load waypoints into list of Waypoints
            @param[in] file: .yaml mission file
            @param[out] self.waypoints: list of ROS waypoint msgs
            """

        with open(file, 'r') as f:

            # Load mission yaml file for parsing
            doc = yaml.load(f)
            self.mission = doc['title']
            rospy.loginfo("Loading mission " + self.mission)
            self.waypoints = [None] * len(doc['waypoints'])

            # Create ROS Waypoint, add to waypoints attribute
            for i in range(len(doc['waypoints'])):
                msg = Waypoint()
                msg.header = Header(frame_id=doc['waypoints'][i]['frame'])
                msg.index = doc['waypoints'][i]['index']
                msg.behavior = doc['waypoints'][i]['behavior']
                msg.direction = doc['waypoints'][i]['direction']
                msg.autocontinue = bool(doc['waypoints'][i]['autocontinue'])
                self.waypoints[i] = msg
                rospy.loginfo("| Loaded waypoint " + str(self.waypoints[i].index))

            return

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
        rospy.loginfo("Starting mission: " + self.mission)
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
        rospy.loginfo("Loading behaviors")
        if rospy.has_param('/behaviors'):
            temp_list = rospy.get_param('/behaviors')
        else:
            rospy.logerr("Error: no /behaviors/ parameters - did you set up the parameter server?")
            rospy.signal_shutdown('no /behaviors/ parameters')
            return 1

        if len(temp_list) > 0:
            # Populate behavior list with parameter-defined behaviors
            for behavior in temp_list:
                self.behaviors[behavior] = temp_list[behavior]
                rospy.loginfo("| Loaded behavior " + behavior + " with id " + str(temp_list[behavior]))
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
        if not len(self.waypoints) == wp_index + 1:
            msg.waypoint2 = self.waypoints[wp_index + 1]
        else:
            msg.waypoint2 = msg.waypoint1
        rospy.loginfo("| Starting behavior: " + msg.waypoint1.behavior + ", index: " + str(msg.label))

        msg_state = String()
        msg_state.data = str(msg.label)
        self.state_pub.publish(msg_state)
        self.behavior_pub.publish(msg)
        return

    def end_behavior(self, wp_index):
        """ @brief ends execution of waypoint behavior
            @param[in] wp_1: Waypoint to end
            """

        # TODO:run behavior end function
        self.state_pub.publish(String('safety'))
        rospy.loginfo("| Ending behavior: " + self.waypoints[wp_index].behavior)
        if not self.waypoints[wp_index].autocontinue:
            pass
            #TODO Add pause for user input here

        return

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    m1 = MissionPlanner()
    m1.run()
