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

class CutBehavior {

public:
  explicit CutBehavior();
  int runInit(const cut_mission::WaypointPairLabeled& p, std::string f);
  int runInit(const cut_mission::WaypointPairLabeled& p, std_msgs::String f);
  int runHalt();
  void spin();
  bool arrivedAtPoint();

private:
  ros::NodeHandle n;
  ros::Rate rate;
  ros::Subscriber waypoint_sub;
  ros::Publisher twist_pub;
  ros::Publisher path_pub;
  bool is_running;
  std::string frame_id;
  std::string label;
  laser_assembler::AssembleScans buildcloud_srv;
  ros::ServiceClient srv_client;

  void scanCB(const sensor_msgs::LaserScan& msg);
  void waypointPairCB(const cut_mission::WaypointPairLabeled& msg);
};

#endif
