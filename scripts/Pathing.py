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
		self.threshold = 1.0	# Dist from waypoint to count as achieved (m)
		self.speed = 0.75		# Percentage velocity (1)
		rospy.spin()

	def checkArrival(self, req):
		# checks whether the tractor is close enough to the 2nd waypoint
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
		self.angular = quaternion_to_euler(self.angular)

		# Note: ROS tf function doesn't output the correct angle
		# self.angular = tf.transformations.euler_from_quaternion(self.angular)

		# Compute direction to travel next
		vector = self.waypointsToVectors(waypoint1, waypoint2)

		# Initialize output message
		newTwist = Twist()

		# Scale twist based on distance from waypoint (commented out for now)
		# newTwist.linear.x = math.sqrt((self.linear[0] - waypoint2.point.x)**2 + (self.linear[1]-waypoint2.point.y)**2) / 5
		# if(newTwist.linear.x > self.speed):
		# 	newTwist.linear.x = self.speed

		# Set twist to constant velocity
		newTwist.linear.x = 1

		# Compute difference between current angle and angle towards waypoint
		# Note: negative angle indicates a right turn is needed, positive
		# indicates a left turn is needed
		angle = self.angular[2] - math.degrees(math.atan2(vector[1], vector[0]))

		# Handle the computation errors that occur when angles "jump" around 360
		if angle > 270:
			angle = angle - 360

		elif angle < -270:
			angle = angle + 360

		# Apply proportional control to angle input
		kp=1 # proportional constant
		desired_angle = angle*kp

		# Need to remap angle to -1 to 1 for tractor input
		angular_cmd = reMap(desired_angle, -45, 45, -1, 1)
		newTwist.angular.z = angular_cmd

		return newTwist

	def waypointsToVectors(self, waypoint1, waypoint2):
		# gets vector from tractor odom to the next waypoint
		# param: waypoint 1 and 2
		# returns a vector with the base at the current odom and the tip at the next waypoint

		vector = [None] * 2
		vector[0] = waypoint2.point.x - self.linear[0]
		vector[1] = waypoint2.point.y - self.linear[1]

		return vector

def quaternion_to_euler((w, x, y, z)):
	# convert from quaternion to euler angle
	# param: (w,x,y,z) tuple of quaternion angle
	# returns X,Y,Z tuple of roll, pitch, and yaw in degrees (RH coordinates)

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def reMap(value, minInput, maxInput, minOutput, maxOutput):
	# remap value to a different scaling. Will also cap value if it is out of
	# range
	# param: value - value to be remapped
	# 		 minInput - min value of value
	#		 maxInput - max value of value
	# 		 minOutput - min value of remap range
	#		 maxOutput - max value of remap range

	value = maxInput if value > maxInput else value
	value = minInput if value < minInput else value

	inputSpan = maxInput - minInput
	outputSpan = maxOutput - minOutput

	scaled_value = float(value - minInput) / float(inputSpan)

	return minOutput + (scaled_value * outputSpan)


if __name__ == "__main__":
	ex = Pathing()
