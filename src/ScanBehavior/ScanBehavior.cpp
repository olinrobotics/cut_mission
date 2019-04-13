#include <fstream>
#include "ScanBehavior.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cut_mission/WaypointPairLabeled.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <state_controller/TwistLabeled.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

ScanBehavior::ScanBehavior()
 : rate(ros::Rate(5))
 , scan_sub(n.subscribe("/scan", 1, &ScanBehavior::ScanBehavior::scanCB, this))
 , waypoint_sub(n.subscribe("/temp_behavior", 1, &ScanBehavior::ScanBehavior::waypointPairCB, this))
 , twist_pub(n.advertise<state_controller::TwistLabeled>("/state_controller/cmd_behavior_twist", 1))
 , filename("")
 , is_running(false)
 , frame_id("None")
 , label("scan"){

   // Ensure filename parameter is properly loaded
   n.getParam("/scan_behavior/file_name", filename);
   if (filename == "") {
     ROS_ERROR("ScanBehavior: cannot find /scan_behavivor/file_name parameter - did you run the launch file?");
     ros::shutdown();
   }
   file = std::ofstream();
   projecter = new laser_geometry::LaserProjection();
   tf_listener = new tf::TransformListener();
 }

void ScanBehavior::scanCB(const sensor_msgs::LaserScan& msg) {
  // If behavior is running, write scan to file
  // Transformations done referencing https://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
  // https://wiki.ros.org/tf/Tutorials/Using%20Stamped%20datatypes%20with%20tf%3A%3AMessageFilter
  if (is_running) {
    sensor_msgs::PointCloud cloud;
    try {
      projecter->transformLaserScanToPointCloud("odom", msg, cloud, *tf_listener);
    } catch (tf::TransformException& e) {
        std::cout<<e.what()<<std::endl;
        return;
    }

    std::cout<<cloud.points[0].x<<std::endl;

    for (int i = 0; i<msg.ranges.size(); i++) {
      file<<msg.ranges[i]<<" ";
    }
  file<<std::endl;
  return;
  }
}

void ScanBehavior::waypointPairCB(const cut_mission::WaypointPairLabeled& msg) {
  // If msg has behavior label, start the behavior
  if (label == msg.waypoint1.behavior) {
    if (runInit(msg, "")) ROS_WARN("%s - Unable to initialize behavior", label.c_str());
  } else {
    if (runHalt()) ROS_WARN("%s - Unable to halt behavior", label.c_str());
  }
}

int ScanBehavior::runInit(const cut_mission::WaypointPairLabeled& p, std_msgs::String f) {
  // Call runInit with string data from ROS std_msgs::String
  return runInit(p, f.data);
}

int ScanBehavior::runInit(const cut_mission::WaypointPairLabeled& p, const std::string f="") {
  // Starts behavior if not running
  if (is_running) {
    ROS_WARN("%s - already running a scan, cannot complete request", label.c_str());
    return 1;
  }
  ROS_INFO("%s - starting behavior", label.c_str());

  // Open filename param if no file given
  if (f=="") {
    file.open(filename);
    ROS_WARN("%s - no filename given, using /scan_behavior/file_name parameter", label.c_str());
    ROS_INFO("%s - opening file for logging:\n%s", label.c_str(), filename.c_str());
  }
  else {
    file.open(f);
    ROS_INFO("ScanBehavior: opening %s for logging", f.c_str());
  }

  is_running = true;
  file<<"LOCALIZED SCAN DATA STORAGE FILE - GENERATED BY SCANBEHAVIOR.CPP"<<std::endl;
  return 0;
}

void ScanBehavior::spin() {
  // Keeps node updating at rate attr. frequency

  while (n.ok()) {
    if (is_running) {
      if (arrivedAtPoint()) runHalt();
      else {
        // Get vector from Nathan's node
        auto msg = state_controller::TwistLabeled();
        msg.label.data = label;
        twist_pub.publish(msg);
      }
    }
    ros::spinOnce();  // Update callbacks
    rate.sleep();     // Standardize output rate
  }
}

int ScanBehavior::runHalt() {
  // Stops behavior if running
  if (!is_running) {
    ROS_WARN("%s - not running a scan, nothing to halt", label.c_str());
    return 1;
  }
  ROS_INFO("%s - halting behavior", label.c_str());
  is_running = false;
  file.close();
  return 0;
}

bool ScanBehavior::arrivedAtPoint() {
  // Compares current position to waypoint
  if (1) return false;
  else return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ScanBehavior");
  ScanBehavior scan;
  scan.spin();
}
