#include <fstream>
#include "ScanBehavior.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ScanBehavior::ScanBehavior()
 : rate(ros::Rate(2))
 , scan_sub(n.subscribe("/scan", 1, &ScanBehavior::ScanBehavior::scanCB, this))
 , filename("")
 , is_running(false) {
   n.getParam("/scan_behavior/file_name", filename);
   if (filename == "") {
     ROS_ERROR("ScanBehavior: cannot find /scan_behavivor/file_name parameter - did you run the launch file?");
     ros::shutdown();
   }
   file = std::ofstream();
   setup();
   shutdown();
 }

void ScanBehavior::scanCB(const sensor_msgs::LaserScan& msg) {

  if (is_running) file << "Hello" << std::endl;
  return;
}

void ScanBehavior::setup() {
  file.open(filename);
  file << "Hello\nGoodbye" << std::endl;
}

void ScanBehavior::shutdown() {
  file.close();
}
int main(int argc, char** argv) {

  ros::init(argc, argv, "ScanBehavior");
  ScanBehavior scan;
  ros::spin();
}
