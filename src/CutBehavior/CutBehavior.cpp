#include <fstream>
#include "ScanBehavior.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <state_controller/TwistLabeled.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <laser_assembler/AssembleScans.h>
#include <cut_mission/CheckArrival.h>
#include <cut_mission/GetCurrTwist.h>
#include <cut_mission/WaypointPairLabeled.h>
#include <blade_nav/HitchPath.h>

CutBehavior::CutBehavior()
 : rate(ros::Rate(15))
 , waypoint_sub(n.subscribe("/waypoints", 1, &ScanBehavior::ScanBehavior::waypointPairCB, this))
 , twist_pub(n.advertise<state_controller::TwistLabeled>("/twist", 1))
 , path_pub(n.advertise<blade_nav::HitchPath>("/path", 1))
 , cut_client(n.serviceClient<cut_mission::CutPlan>("CutPlan"))
 ,
