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
		self.threshold = 0.5	# Dist from path to count as on path (m)
		self.speed = 0.75		# Percentage velocity (1)
		rospy.spin()

	def checkArrival(self, req):
		#linear doesn't have x????
		self.linear, self.angular = self.listener.lookupTransform("/odom", "/base_link", rospy.Time())
		vector = self.odomToWaypoint(req.waypoint2)
		if(vector.x**2 + vector.y**2 < self.threshold**2):
			return Bool(True)
		else:
			return Bool(False)

	def odomToWaypoint(self, waypoint2):
		vector = Point()
		vector.y = waypoint2.point.y - self.linear[1]
		vector.x = waypoint2.point.x - self.linear[0]
		rospy.loginfo(vector)
		return vector

	def getCurrentTwist(self, req):
		# Publish twist msg based on current tractor position
		waypoint1 = req.waypoint1
		waypoint2 = req.waypoint2
		self.linear, self.angular = self.listener.lookupTransform("/odom", "/base_link", rospy.Time())
		rospy.loginfo(self.angular[2])
		rospy.loginfo(self.linear)
		vector = self.waypointsToVectors(waypoint1, waypoint2)
		newTwist = Twist()
		newTwist.linear.x = math.sqrt((self.linear[0] - waypoint2.point.x)**2 + (self.linear[1]-waypoint2.point.y)**2) / 5
		if(newTwist.linear.x > self.speed):
			newTwist.linear.x = self.speed
		angle = math.atan2(vector[1], vector[0]) - self.angular[2] * math.pi
		rospy.loginfo(angle)
		angle = angle / (math.pi / 4)
		if(angle > math.pi):
			angle = angle - 2*math.pi
		elif(angle < -math.pi):
			angle = angle + 2*math.pi
		if(angle > 1):
			angle = 1
		elif(angle < -1):
			angle = -1
		newTwist.angular.z = angle
		rospy.loginfo(newTwist)
		return newTwist

	def waypointsToVectors(self, waypoint1, waypoint2):
		# Gets vector for tractor to travel on
		# If on line, get vector along line
		# If not on line, get vector towards line
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

		x1 = waypoint1.point.x
		x2 = waypoint2.point.x
		y1 = waypoint1.point.y
		y2 = waypoint2.point.y
		a = y1 - y2
		b = x2 - x1
		c = x1*y2 - x2*y1
		if (a == b == 0): distance = 0
		else: distance = (a*self.linear[0] + b*self.linear[1] + c) / (math.sqrt(a**2 + b**2))
		rospy.loginfo(distance)
		return distance

if __name__ == "__main__":
	ex = Pathing()
