#ifndef SCAN_BEHAVIOR_H
#define SCAN_BEHAVIOR_H

#include <string>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <laser_assembler/AssembleScans.h>
#include <cut_mission/CheckArrival.h>
#include <cut_mission/GetCurrTwist.h>
#include <cut_mission/WaypointPairLabeled.h>

class ScanBehavior {

public:
  explicit ScanBehavior();
  int runInit(const cut_mission::WaypointPairLabeled& p, std::string f);
  int runInit(const cut_mission::WaypointPairLabeled& p, std_msgs::String f);
  int runHalt();
  void spin();
  bool arrivedAtPoint();

private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
  ros::Subscriber waypoint_sub;
  ros::Publisher twist_pub;
  ros::ServiceClient assemble_client;
  ros::ServiceClient twist_client;
  ros::ServiceClient arrive_client;
  ros::Rate rate;
  laser_geometry::LaserProjection* projector;
  tf::TransformListener* tf_listener;
  std::string filename;
  std::string datatype;
  std::ofstream file;
  bool is_running;
  std::string frame_id;
  std::string label;
  cut_mission::WaypointPairLabeled* waypoints;
  laser_assembler::AssembleScans buildcloud_srv;
  cut_mission::CheckArrival      arrive_srv;
  cut_mission::GetCurrTwist      twist_srv;


  void scanCB(const sensor_msgs::LaserScan& msg);
  void waypointPairCB(const cut_mission::WaypointPairLabeled& msg);
};

#endif
