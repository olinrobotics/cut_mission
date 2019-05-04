#ifndef CUT_BEHAVIOR_H
#define CUT_BEHAVIOR_H

#include <string>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cut_mission/WaypointPairLabeled.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <laser_assembler/AssembleScans.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "KdTree.h"
#include <cut_mission/CheckArrival.h>
#include <cut_mission/GetCurrTwist.h>
#include <cut_mission/WaypointPairLabeled.h>

class CutBehavior {

public:
  explicit CutBehavior();
  void spin();

private:
  ros::NodeHandle n;
  ros::Subscriber hitch_pose_sub;
  ros::Subscriber hitch_path_sub;
  ros::Subscriber waypoint_sub;
  ros::Publisher twist_pub;
  ros::ServiceClient twist_client;
  ros::ServiceClient arrive_client;
  ros::ServiceClient cut_client;

  ros::Rate rate;

  cut_mission::WaypointPairLabeled* waypoints;
  std::string label; // Behavior label

  // ROS Messages
  geometry_msgs::Pose hitch_pose;
  nav_msgs::Path hitch_path;

  // Other Variables
  bool is_running;
  kd_node_t * root; // Root node for kd tree (for looking up nearest hitch)
  KdTree * kd;

  // Callback Functions
  void hitchCB(const geometry_msgs::Pose& msg);
  void pathCB(const nav_msgs::Path& msg);
  void waypointPairCB(const cut_mission::WaypointPairLabeled& msg);

  // Init-Halt Functions
  int runInit(const cut_mission::WaypointPairLabeled& p);
  int runHalt();

  // Main Functions
  struct kd_node_t * pathToArray(nav_msgs::Path path);
  double getNearestHeight();
};

#endif

//   laser_geometry::LaserProjection* projecter;
  // tf::TransformListener tf_listener;
// nav_msgs::Odometry tractor_odom;
// void odomCB(const nav_msgs::Odometry& msg);

//   std::string filename;
//   std::string datatype;
//   std::ofstream file;
//   bool is_running;
//   std::string frame_id;
//   std::string label;
//   laser_assembler::AssembleScans buildcloud_srv;
//   ros::ServiceClient srv_client;
//
//   void scanCB(const sensor_msgs::LaserScan& msg);
//   void waypointPairCB(const cut_mission::WaypointPairLabeled& msg);
