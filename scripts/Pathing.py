#!/usr/bin/env python
import rospy
import math
from cut_mission.msg import Waypoint, WaypointPairLabeled
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import tf
from cut_mission.srv import *


class Pathing():
	def __init__(self):
		rospy.init_node("pathing")
		self.s = rospy.Service('getCurrentTwist', GetCurrTwist, self.getCurrentTwist)
		self.s = rospy.Service('checkArrival', CheckArrival, self.checkArrival)
		waypoint1 = None
		waypoint2 = None
		self.listener = tf.TransformListener()
		self.threshold = 1.0	# Dist from path to count as on path (m)
		self.speed = 0.75		# Percentage velocity (1)
		rospy.spin()

	def checkArrival(self, req):
		# checks whether the tractor is clone enough to the 2nd waypoint
		# param: req, the pair of waypoints
		# returns a ROS Bool
		self.linear, self.angular = self.listener.lookupTransform("/odom", "/base_link", rospy.Time())
		vector = self.odomToWaypoint(req.waypoint2)
		if(vector.x**2 + vector.y**2 < self.threshold**2): return Bool(True)
		else: return Bool(False)

	def odomToWaypoint(self, waypoint2):
		# gives the vector from the odometry to the 2nd waypoint
		# param: 2nd waypoint
		# returns a point vector
		vector = Point()
		vector.y = waypoint2.point.y - self.linear[1]
		vector.x = waypoint2.point.x - self.linear[0]
		return vector

	def getCurrentTwist(self, req):
		# Publish twist msg based on current tractor position
		# param: the pair of waypoints
		# returns a twist
		# notes: the convert to ackermann takes a number from -1 to 1, so the final turn must be converted from radians to -1 to 1
		# ex: if the radians is pi / 4, you want to pass 1 because 1 corresponds to 45 degrees
		waypoint1 = req.waypoint1
		waypoint2 = req.waypoint2
		self.linear, self.angular = self.listener.lookupTransform("/odom", "/base_link", rospy.Time())
		self.angular = tf.transformations.euler_from_quaternion(self.angular)
		vector = self.waypointsToVectors(waypoint1, waypoint2)
		newTwist = Twist()
		newTwist.linear.x = math.sqrt((self.linear[0] - waypoint2.point.x)**2 + (self.linear[1]-waypoint2.point.y)**2) / 5
		if(newTwist.linear.x > self.speed):
			newTwist.linear.x = self.speed
		angle = math.atan2(vector[1], vector[0]) - self.angular[2]
		# angle = angle / (math.pi / 4)
		if(angle > math.pi):
			angle = angle - 2*math.pi
		elif(angle < -math.pi):
			angle = angle + 2*math.pi
		if(angle > 1):
			angle = 1
		elif(angle < -1):
			angle = -1
		newTwist.angular.z = angle
		return newTwist

	def waypointsToVectors(self, waypoint1, waypoint2):
		# gets vector from tractor odom to the point 1 meter in front on the line.
		# if the tractor is off the line, it will point towards the point 1 meter forward from the perpendicular point of the line
		# param: waypoint 1 and 2
		# returns a vector
		vector = [None] * 2
		normVec = [None] * 2
		distance = self.distanceToLine(waypoint1,waypoint2)
		normVec[1] = waypoint2.point.y - waypoint1.point.y
		normVec[0] = waypoint2.point.x - waypoint1.point.x

		normVec = [normVec[0] / math.sqrt(normVec[0]**2 + normVec[1]**2), normVec[1] / math.sqrt(normVec[0]**2 + normVec[1]**2)]
		vector[1] = normVec[1] - normVec[0]*distance
		vector[0] = normVec[0] + normVec[1]*distance
		return vector

	def distanceToLine(self, waypoint1, waypoint2):
		# gives distance to line between waypoints
		# param: waypoints 1 and 2
		# returns a distance
		x1 = waypoint1.point.x
		x2 = waypoint2.point.x
		y1 = waypoint1.point.y
		y2 = waypoint2.point.y
		a = y1 - y2
		b = x2 - x1
		c = x1*y2 - x2*y1
		if (a == b == 0): distance = 0
		else: distance = (a*self.linear[0] + b*self.linear[1] + c) / (math.sqrt(a**2 + b**2))
		return distance

if __name__ == "__main__":
	ex = Pathing()
