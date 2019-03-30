import rospy
import math

class Pathing():
	def __init__(self):
		rospy.init_node("pathing")
		self.waypointSub = rospy.Subscriber("waypoints", WaypointMSG, self.waypointCB)
		self.odomSub = rospy.Subscriber("odom_topic", PoseStamped, self.odomCB)
		self.twistPub = rospy.Publisher("clapback_twist", TwistStamped, queue_size=10)
		self.statusPub = rospy.Publisher("waypoint_status", String?, queue_size=10)
		self.threshold = 0.2
		self.speed = 0.75
		self.waypoints = None
		self.odom = None
	def waypointCB(self, data):
		self.waypoints = data

	def odomCB(self,data):
		self.odom = data

	def go(self):
		while(!self.isItThere()):
			self.passOnTwist()
		self.statusPub.publish()

	def isItThere(self):
		vector = self.calculateVectorFromOdomToWaypoint()
		if(vector.x**2 + vector.y**2 + vector.z**2 < self.threshold**2):
			return True
		else:
			return False

	def passOnTwist(self):
		vector = self.calculateVectorFromOdomToWaypoint()
		newTwist = Twist()
		newTwist.linear.x = self.speed
		newTwist.angular.z = math.atan2(vector.y/vector.x)
		twistPub.publish(newTwist)

def go(waypoints):
	# do while loop
	# while?(is it there)
	#	publish "I'm at waypoint H"
	# 	increment waypoint
	# passonTwist
	# while there are still waypoints left
	# done?
	# returns something idk
def isitThere(currentOdom, nextWaypoint):
	# calcVecOdom
	# is the vec less than a specified threshold?
	# returns boolean
def passOnTwist(currentOdom, nextWaypoint):
	# something like:
	# calculate the vector from odom to the waypoint
	# maybe need to specify speed, but how?
	# publish the twist? or ackermann to the MainState
	# returns nothing
def calculateVectorFromOdomToWaypoint(currentOdom,nextWaypoint):
	# vector math