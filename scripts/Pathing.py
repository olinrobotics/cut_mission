#!/usr/bin/env python
import rospy
import math
from cut_mission.msg import Waypoint, WaypointPairLabeled
from geometry_msgs.msg import Twist
import tf
from cut_mission.srv import *


class Pathing():
	def __init__(self):
		rospy.init_node("pathing")
		self.s = rospy.Service('getCurrentTwist', GetCurrTwist, self.getCurrentTwist)
		self.s = rospy.Service('checkArrival', checkArrival, checkArrival)
		waypoint1 = None
		waypoint2 = None
		self.listener = tf.TransformListener()
		self.threshold = 0.2	# Dist from path to count as on path (m)
		self.speed = 0.75		# Percentage velocity (1)
		rospy.spin()

	def isItThere(self, req):
		vector = self.OdomToWaypoint(req.wp2)
		if(vector.x**2 + vector.y**2 + vector.z**2 < self.threshold**2):
			return checkArrivalResponse(True)
		else:
			return checkArrivalResponse(False)

	def odomToWaypoint(self, waypoint2):
		vector.y = waypoint2.y - self.odom.y
		vector.x = waypoint2.x - self.odom.x
		return vector

	def getCurrentTwist(self, req):
		# Publish twist msg based on current tractor position
		waypoint1 = req.a
		waypoint2 = req.b
		self.linear, self.angular = self.tf.lookupTransform("/base_link", "/map", rospy.Time())
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
			vector.y = waypoint2.y - self.linear.y
			vector.x = waypoint2.x - self.linear.x
			return vector
		else:
			distance = self.distanceToLine(waypoint1, waypoint2)
			vector.y = waypoint2.y - waypoint1.y + -1*distance * (waypoint2.x - waypoint1.x)
			vector.x = waypoint2.x - waypoint1.x + distance * (waypoint2.y - waypoint1.y)
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

		x1 = waypoint1.x
		x2 = waypoint2.x
		y1 = waypoint1.y
		y2 = waypoint2.y
		a = y1 - y2
		b = x2 - x1
		c = x1*y2 - x2*y1
		distance = a*self.linear.x + b*self.linear.y + c / (math.sqrt(a**2 + b**2))
		return distance

if __name__ == "__main__":
	ex = Pathing()
