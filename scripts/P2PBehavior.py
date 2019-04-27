#!/usr/bin/env python

import rospy
import Pathing
from cut_mission.msg import Waypoint, WaypointPairLabeled
from geometry_msgs.msg import Twist
from state_controller.msg import TwistLabeled
from cut_mission.srv import *
from std_msgs.msg import String

class P2PBehavior:
    ''' @brief Behavior class for basic waypoint navigation

        Behavior class - Upon recieving a WaypointPairLabeled message
        on the /mission_planner/out_behavior topic,
        '''

    def __init__(self):

        # Set up basic attributes
        self.is_active = False      # Whether or not actively executing behavior
        self.wp_1 = None            # Starting waypoint
        self.wp_2 = None            # Ending waypoint
        self.arrival_margin = 0.1   # Circular region of error around waypoint (m)
        self.label = "p2p"          # Behavior message label

        # Set up ros constructs
        rospy.init_node('p2p_behavior')
        rospy.Subscriber("/waypoints", WaypointPairLabeled, self.waypointpair_cb)
        self.twist_pub = rospy.Publisher('/state_controller/cmd_behavior_twist', TwistLabeled, queue_size=1)
        self.rate = rospy.Rate(10)
        pass

    def waypointpair_cb(self, msg):
        ''' @brief callback func for activating behavior
            @param msg[WaypointPairLabeled]: Waypoints for navigation
            '''

        if msg.label == self.label:
            if self.is_active:
                rospy.logerr('P2PBehavior: Cannot execute new command, already running')
            else:
                self.wp_1 = msg.waypoint1
                self.wp_2 = msg.waypoint2
                self.is_active = True

    def check_arrival(self):
        ''' @brief Check for arrival at attr waypoint_2
            return [bool] have we arrived
            '''
        rospy.wait_for_service('checkArrival')
        try:
            checkArrival = rospy.ServiceProxy('checkArrival', CheckArrival)
            response = checkArrival(self.wp_1, self.wp_2)
            return response.arrived
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def reset_behavior(self):
        ''' @brief clears metadata, prepares for next activation
            '''
        self.is_active = False
        self.wp_1 = None
        self.wp_2 = None
        return

    def run(self):
        ''' @brief run main behavior loop
            '''

        while not rospy.is_shutdown():
            if self.is_active:
                if self.check_arrival():
                    msg = TwistLabeled()
                    msg.label = String("p2p")
                    rospy.wait_for_service('getCurrentTwist')
                    try:
                        getCurrentTwist = rospy.ServiceProxy('getCurrentTwist', GetCurrTwist)
                        response = getCurrentTwist(self.wp_1, self.wp_2)
                        msg.twist = response.vector
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                    self.twist_pub.publish(msg)

                else:
                    self.reset_behavior()

            self.rate.sleep()

if __name__ == '__main__':
    rospy.set_param('topic', '/mission_planner/out_behavior')
    behavior = P2PBehavior()
    behavior.run()
