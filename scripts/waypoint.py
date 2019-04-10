import rospy
import math

class Pathing():
	def __init__(self, waypoint1, waypoint2):
		rospy.init_node("pathing")
		self.waypoint1 = waypoint1
		self.waypoint2 = waypoint2
		self.odomSub = rospy.Subscriber("odom_topic", PoseStamped, self.odomCB)
		self.twistPub = rospy.Publisher("clapback_twist", TwistStamped, queue_size=10)
		self.statusPub = rospy.Publisher("waypoint_status", String?, queue_size=10)
		self.threshold = 0.2
		self.speed = 0.75
		self.waypoints = None
		self.odom = None

	def odomCB(self,data):
		self.odom = data

	def go(self):
		while(!self.isItThere()):
			self.passOnTwist()
		self.statusPub.publish()

	def isItThere(self):
		vector = self.OdomToWaypoint()
		if(vector.x**2 + vector.y**2 + vector.z**2 < self.threshold**2):
			return True
		else:
			return False

	def odomToWaypoint(self):
		vector.y = self.waypoint2.y - self.odom.y
		vector.x = self.waypoint2.x - self.odom.x
		return vector

	def passOnTwist(self):
		vector = self.waypointsToVectors()
		newTwist = Twist()
		newTwist.linear.x = self.speed
		newTwist.angular.z = math.atan2(vector.y/vector.x) - self.odom.angular.z
		twistPub.publish(newTwist)

	def waypointsToVectors(self):
		if(self.onTheLine()):
			vector.y = self.waypoint2.y - self.odom.y
			vector.x = self.waypoint2.x - self.odom.x
			return vector
		else:
			distance = self.distanceToLine()
			vector.y = self.waypoint2.y - self.waypoint1.y + -1*distance * (self.waypoint2.x - self.waypoint1.x)
			vector.x = self.waypoint2.x - self.waypoint1.x + distance * (self.waypoint2.y - self.waypoint1.y)	
			return vector	

	
	def onTheLine(self):
		distance = self.distanceToLine()
		if(math.abs(distance) < self.threshold):
			return True
		else:
			return False	

	def distanceToLine(self):
		x1 = self.waypoint1.x
		x2 = self.waypoint2.x
		y1 = self.waypoint1.y
		y2 = self.waypoint2.y
		a = y1 - y2
		b = x2 - x1
		c = x1*y2 - x2*y1
		distance = a*x + b*y + c / (math.sqrt(a**2 + b**2))
		return distance