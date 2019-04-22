#!/usr/bin/env python
import rospy
import math
from cut_mission.msg import Waypoint, WaypointPairLabeled
from geometry_msgs.msg import Twist, Point
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
		self.threshold = 0.2	# Dist from path to count as on path (m)
		self.speed = 0.75		# Percentage velocity (1)
		rospy.spin()

	def checkArrival(self, req):
		#linear doesn't have x????
		self.linear, self.angular = self.listener.lookupTransform("/base_link", "/odom", rospy.Time())
		vector = self.odomToWaypoint(req.waypoint2)
		if(vector.x**2 + vector.y**2 + vector.z**2 < self.threshold**2):
			return checkArrivalResponse(True)
		else:
			return checkArrivalResponse(False)

	def odomToWaypoint(self, waypoint2):
		vector = Point()
		vector.y = waypoint2.point.y - self.linear.y
		vector.x = waypoint2.point.x - self.linear.x
		return vector

	def getCurrentTwist(self, req):
		# Publish twist msg based on current tractor position
		waypoint1 = req.waypoint1
		waypoint2 = req.waypoint2
		self.linear, self.angular = self.listener.lookupTransform("/base_link", "/odom", rospy.Time())
		vector = self.waypointsToVectors(waypoint1, waypoint2)
		newTwist = Twist()
		newTwist.linear.x = self.speed
		newTwist.angular.z = math.atan2(vector.y/vector.x) - self.angular.z
		return newTwist

	def waypointsToVectors(self, waypoint1, waypoint2):
		# Gets vector for tractor to travel on
		# If on line, get vector along line
		# If not on line, get vector towards line
		if(self.onTheLine(waypoint1, waypoint2)):
			vector.y = waypoint2.point.y - self.linear.y
			vector.x = waypoint2.point.x - self.linear.x
			return vector
		else:
			distance = self.distanceToLine(waypoint1, waypoint2)
			vector.y = waypoint2.point.y - waypoint1.point.y + -1*distance * (waypoint2.point.x - waypoint1.point.x)
			vector.x = waypoint2.point.x - waypoint1.point.x + distance * (waypoint2.point.y - waypoint1.point.y)
			return vector

	def onTheLine(self, waypoint1, waypoint2):
		# Returns bool if tractor within threshold of line

		distance = self.distanceToLine()
		if(math.abs(distance) < self.threshold):
			return True
		else:
			return False

	def distanceToLine(self, waypoint1, waypoint2):
		# gives distance to line between waypoints

		x1 = waypoint1.point.x
		x2 = waypoint2.point.x
		y1 = waypoint1.point.y
		y2 = waypoint2.point.y
		a = y1 - y2
		b = x2 - x1
		c = x1*y2 - x2*y1
		distance = a*self.linear.x + b*self.linear.y + c / (math.sqrt(a**2 + b**2))
		return distance

if __name__ == "__main__":
	ex = Pathing()
