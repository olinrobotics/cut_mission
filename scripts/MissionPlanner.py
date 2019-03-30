#!/usr/bin/env python

import rospy
import yaml
from std_msgs.msg import String, Int16

class MissionPlanner:
    """ @brief executes mission from mission file
        Reads mission from a .yaml mission file (template in /missions),
        Sequentially passes between waypoints, executing behaviors accompanying
        each mission path segment, until reaching an end behavior.
        """

    def __init__(self):

        # Set up mission
        self.load_mission('../missions/single_cut.yaml')
        self.print_mission()
        self.curr_waypoint = None

        # Set up ros stuff
        rospy.init_node('mission_planner')
        self.state_pub = rospy.Publisher('/state_controller/cmd_state', String, queue_size=1)
        rospy.Subscriber('/mission_planner/wp_pass', Int16, self.wp_pass_cb)

        self.start_mission()

    def wp_pass_cb(self, msg):
        """ @brief end segment if reached wp is next in mission
            """

        if msg.data == self.curr_waypoint.get_index() + 1:
            self.next_behavior()

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
        self.setup_behavior(self.current_waypoint)
        self.run_behavior(self.current_waypoint)
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

    def next_behavior(self):
        """ @brief end current behavior, setup & start next behavior
            """
        self.end_segment(self.current_waypoint)

        try:
            self.current_waypoint = self.waypoints[self.current_waypoint.index + 1]
            self.setup_segment(self.current_waypont)
            self.run_segment(self.current_waypoint)
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
        # TODO:run behavior start function
        self.curr_waypoint = waypoint
        return

    def run_behavior(self, waypoint):
        pass

    def end_behavior(self, waypoint):
        """ @brief ends execution of waypoint behavior
            @param[in] wp_1: Waypoint to end
            """

        # TODO:run behavior end function
        self.state_pub.publish(String('safety'))
        if not self.curr_wp.get_autocontinue():
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
