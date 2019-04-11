#ifndef SCAN_BEHAVIOR_H
#define SCAN_BEHAVIOR_H

#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

class ScanBehavior {

public:
  explicit ScanBehavior();
private:
  ros::NodeHandle n;
  ros::Subscriber scan_sub;
  ros::Rate rate;
  std::string filename;
  std::ofstream file;
  bool is_running;

  void scanCB(const sensor_msgs::LaserScan& msg);
  void setup();
  void shutdown();
};

#endif
