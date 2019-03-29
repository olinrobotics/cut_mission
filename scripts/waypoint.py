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
	# do trig to calc the steering angle
	# maybe need to specify speed, but how?
	# publish the twist? or ackermann to the MainState
	# returns nothing
def calculateVectorFromOdomToWaypoint(currentOdom,nextWaypoint):
	# vector math