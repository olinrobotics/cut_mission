#!/usr/bin/env python

import rospy
import Pathing
from cut_mission.msg import Waypoint, WaypointPairLabeled
from geometry_msgs.msg import Twist, Vector3, Point
from state_controller.msg import TwistLabeled
from cut_mission.srv import *
from std_msgs.msg import String, Bool, ColorRGBA, Header
from visualization_msgs.msg import Marker
from copy import deepcopy

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
        self.arrived_pub = rospy.Publisher('/mission_planner/in_behavior', String, queue_size=1)
        self.line_pub = rospy.Publisher('/line_vis', Marker, queue_size=10)
        self.end_pub = rospy.Publisher('/endpoint_vis', Marker, queue_size=10)
        self.start_pub = rospy.Publisher('/startpoint_vis', Marker, queue_size=10)  
        self.rate = rospy.Rate(10)

        pass

    def waypointpair_cb(self, msg):
        ''' @brief callback func for activating behavior
            @param msg[WaypointPairLabeled]: Waypoints for navigation
            '''
        if msg.waypoint1.behavior.data == self.label:
            if self.is_active:
                rospy.logerr('P2PBehavior: Cannot execute new command, already running')
            else:
		rospy.loginfo("p2p - executing behavior")
                self.wp_1 = msg.waypoint1
                self.wp_2 = msg.waypoint2
                self.is_active = True
        #Visualization
        if bool(rospy.get_param('~visualization','False')):
           self.visualize()
        else:
            self.line_pub.publish(Marker())
            self.end_pub.publish(Marker())
            self.start_pub.publish(Marker())

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
        emptyTwist = TwistLabeled()
        emptyTwist.label = String("p2p")
        self.twist_pub.publish(emptyTwist)
        self.is_active = False
        self.wp_1 = None
        self.wp_2 = None
        return

    def visualize(self):
        lineVis = Marker() # Marker to visualize used lidar pts
        lineVis.header.frame_id = "odom" # Publishes it to the laser link, idk if it should be changed
        lineVis.header.stamp = rospy.Time.now()
        endVis = deepcopy(lineVis)
        startVis = deepcopy(lineVis)
        lineVis.points.append(Point(self.wp_1.point.x,self.wp_1.point.y,self.wp_1.point.z))
        startVis.points.append(Point(self.wp_1.point.x,self.wp_1.point.y,self.wp_1.point.z))
        lineVis.points.append(Point(self.wp_2.point.x,self.wp_2.point.y,self.wp_2.point.z))
        endVis.points.append(Point(self.wp_2.point.x,self.wp_2.point.y,self.wp_2.point.z))
        lineVis.color = ColorRGBA(1,1,1,1) # Color is blue
        startVis.color = ColorRGBA(1,0,0,1) # Color is blue
        endVis.color = ColorRGBA(0,1,0,1) # Color is blue
        lineVis.type = 4 # makes it a list of points
        startVis.type = 8
        endVis.type = 8
        endVis.scale.x = 0.3
        endVis.scale.y = 0.3
        startVis.scale.x = 0.3
        startVis.scale.y = 0.3
        lineVis.scale.x = 0.05 # scale is about 2 cm
        lineVis.scale.y = 0.05
        self.line_pub.publish(lineVis)
        self.end_pub.publish(endVis)
        self.start_pub.publish(startVis)

    def run(self):
        ''' @brief run main behavior loop
            '''

        while not rospy.is_shutdown():
            if self.is_active:
                if self.check_arrival() == Bool(False):
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
                    self.arrived_pub.publish(String("p2p"))

            self.rate.sleep()

if __name__ == '__main__':
    rospy.set_param('topic', '/mission_planner/out_behavior')
    behavior = P2PBehavior()
    behavior.run()
