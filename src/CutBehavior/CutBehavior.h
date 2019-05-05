#ifndef CUT_BEHAVIOR_H
#define CUT_BEHAVIOR_H

#include <string>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Standard ROS Messages
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>                    // For read/write blade paths

// Custom ROS Messages
#include <state_controller/TwistLabeled.h>    // For tractor cmds to sc

// ROS services
#include <cut_mission/GetCurrTwist.h>
#include <cut_mission/CheckArrival.h>
#include <cut_mission/CutPlan.h>
#include <cut_mission/WaypointPairLabeled.h>  // For recieving activate/disactive

#include "KdTree.h"

class CutBehavior {

public:
  explicit CutBehavior();
  int runInit(const cut_mission::WaypointPairLabeled& p);
  int runHalt();
  void spin();
  bool arrivedAtPoint();

private:
  ros::NodeHandle n;
  ros::Rate rate;

  // Publishers & Subscribers
  ros::Subscriber hitch_pose_sub;
  ros::Subscriber waypoint_sub;
  ros::Publisher twist_pub;
  ros::Publisher hitch_pub;

  // Service Clients
  ros::ServiceClient twist_client;
  ros::ServiceClient arrive_client;
  ros::ServiceClient cut_client;

  // Services
  cut_mission::GetCurrTwist twist_srv;
  cut_mission::CheckArrival arrive_srv;
  cut_mission::CutPlan cut_srv;

  // Class Member Variables
  std::string filename;
  cut_mission::WaypointPairLabeled *waypoints;
  std::string label;
  bool is_running;
  geometry_msgs::Pose hitch_pose;
  nav_msgs::Path hitch_path;

  // Class Member Functions
  double getNearestHeight();
  struct kd_node_t * pathToArray(nav_msgs::Path path);
  struct kd_node_t *root;
  KdTree* kd;
  void initPath();

  // Callback Functions
  void scanCB(const sensor_msgs::LaserScan& msg);
  void hitchCB(const geometry_msgs::Pose& msg);
  void waypointPairCB(const cut_mission::WaypointPairLabeled& msg);
};

#endif
