#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String, Int16
from cut_planner.msg import Waypoint, WaypointPairLabeled

class MissionPlanner:
    """ @brief executes mission from mission file
        Reads mission from a .yaml mission file (template in /missions),
        Sequentially passes between waypoints, executing behaviors accompanying
        each mission path segment, until reaching an end behavior.
        """

    def __init__(self):

        # Set up mission
        self.load_mission('../config/p2p_test.yaml')
        self.print_mission()
        self.curr_waypoint = None

        # Set up ros stuff
        rospy.init_node('mission_planner')
        self.state_pub = rospy.Publisher('/state_controller/cmd_state', String, queue_size=1)
        self.wppair_pub = rospy.Publisher('/mission_planner/cmd_wppair', WaypointPairLabeled, queue_size=1)
        rospy.Subscriber('/mission_planner/cmd_behavior', Int16, self.wp_pass_cb)

        self.behaviors = {}
        self.update_behaviors()
        self.start_mission()

    def wp_pass_cb(self, msg):
        """ @brief end segment if reached wp is next in mission
            """
        if msg.data == self.curr_waypoint.get_index() + 1 and len(self.waypoints) > msg.data:
            print("Recieved next waypoint")
            self.next_behavior()
        else:
            print("Recieved incorrect waypoint")

    def load_mission(self, file):
        """ @brief load waypoints into list attr
            @param[in] file: .yaml mission file
            @param[out] self.waypoints: list of waypoint obj
            """

        with open(file, 'r') as f:
            doc = yaml.load(f)
            self.mission = doc['title']
            print ("Loading mission " + self.mission)
            self.waypoints = [None] * len(doc['waypoints'])
            for i in range(len(doc['waypoints'])):
                self.waypoints[i] = Waypoint(doc['waypoints'][i])
                print ("Loaded waypoint " + str(self.waypoints[i].get_index()))

            return

    def start_mission(self, init_wp = 0):
        """ @brief run mission in file from waypoint
            @param[in] file: .yaml mission description file
            @param[in] init_wp: index of starting waypoint; defaults to 0
            """
        self.current_waypoint = self.waypoints[init_wp]
        print("Starting mission: " + self.mission)
        self.setup_behavior(self.current_waypoint)
        self.run_behavior(self.current_waypoint)
        return

    def end_mission(self):
        """ @brief end mission, log necessary information
            """
        self.current_waypoint = None
        return

    def print_mission(self):
        """ @brief prints mission pts @ segments
            """
        try:
            print("Mission: " + self.mission)
            for i in range(len(self.waypoints) - 1):
                print("Execute " + self.waypoints[i].behavior + " from " + str(self.waypoints[i].index) + " to " + str(self.waypoints[i+1].index))
            print("Execute " + self.waypoints[-1].behavior + " at " + str(self.waypoints[-1].index))
        except AttributeError as e:
            print("No mission loaded yet. Error: " + str(e))

    def update_behaviors(self):
        ''' @brief Updates behavior parameters
            updateBehaviors checks the behavior namespace on the parameter server and
            populates behavior_list with the behaviors listed there.
            TODO: Ensure no duplicates in behavior vector
            '''

        temp_list = rospy.get_param('/behaviors')
        if len(temp_list) > 0:
            # Populate behavior list with parameter-defined behaviors
            for behavior in temp_list:
                self.behaviors[behavior] = temp_list[behavior]
                print("Found behavior " + behavior + " with id " + str(temp_list[behavior]))

    def next_behavior(self):
        """ @brief end current behavior, setup & start next behavior
            """
        self.end_behavior(self.current_waypoint)
        try:
            self.current_waypoint = self.waypoints[self.current_waypoint.index + 1]
            self.setup_behavior(self.current_waypoint)
            self.run_behavior(self.current_waypoint)
            return

        except IndexError as e:
            print("Reached end of mission, shutting down mission planner.")
            rospy.signal_shutdown()
            return

    def setup_behavior(self, waypoint):
        """ @brief Runs setup func of waypoint behavior
            @param[in] wp_1: Waypoint to start
            @param[out] curr_waypoint: Waypoint being executed
            """
        msg = String()
        msg.data = waypoint.get_behavior()
        self.state_pub.publish(msg)
        print("Setting up behavior: " + msg.data)
        # TODO:run behavior start function
        self.curr_waypoint = waypoint
        return

    def run_behavior(self, waypoint):
        print("   Running behavior: " + waypoint.get_behavior())
        pass

    def end_behavior(self, waypoint):
        """ @brief ends execution of waypoint behavior
            @param[in] wp_1: Waypoint to end
            """

        # TODO:run behavior end function
        self.state_pub.publish(String('safety'))
        print("    Ending behavior: " + waypoint.get_behavior())
        if not self.current_waypoint.get_autocontinue():
            pass
            #TODO Add pause for user input here

        return

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


class behavior:

    def __init__(self, name):
        pass

    def start_behavior(self):
        pass

    def end_behavior(self):
        pass


class Scan(behavior):

    def __init__(self):
        pass

    def start_behavior(self):
        pass

    def end_behavior(self):
        pass


class Waypoint:
    """ @brief stores information from mission .yaml file
        """

    def __init__(self, yaml_wp):
        """ @brief initialize attributes of waypoint from .yaml dict
            @param[in] yaml_wp: yaml-based dictionary
            """

        self.index = int(yaml_wp['index'])
        self.point = yaml_wp['point']
        self.frame = yaml_wp['frame']
        self.behavior = yaml_wp['behavior']
        self.direction = None if yaml_wp['direction'] == 'None' else yaml_wp['direction']
        self.autocontinue = None if yaml_wp['autocontinue'] == 'none' else yaml_wp['autocontinue']

    def __str__(self):
        return ('Waypoint ' + str(self.index) + "\n" + \
                '       Point: ' + str(self.point['x']) +', '+ str(self.point['y']) +', '+ str(self.point['z']) + "\n" + \
                '       Frame: ' + self.frame + "\n" + \
                '     behavior: ' + self.behavior + "\n" + \
                '   Direction: ' + self.direction + "\n" + \
                'Autocontinue: ' + str(self.autocontinue))

    def get_index(self):
        return self.index

    def get_point(self):
        return self.point

    def get_frame(self):
        return self.frame

    def get_behavior(self):
        return self.behavior

    def get_direction(self):
        return self.direction

    def get_autocontinue(self):
        return self.autocontinue

if __name__ == "__main__":
    m1 = MissionPlanner()
    m1.run()
