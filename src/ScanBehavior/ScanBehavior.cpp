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

ScanBehavior::ScanBehavior()
 : rate(ros::Rate(15))
 , scan_sub    (n.subscribe("/scan", 1, &ScanBehavior::ScanBehavior::scanCB, this))
 , waypoint_sub(n.subscribe("/waypoints", 1, &ScanBehavior::ScanBehavior::waypointPairCB, this))
 , twist_pub   (n.advertise<state_controller::TwistLabeled>("/twist", 1))
 , assemble_client(n.serviceClient<laser_assembler::AssembleScans>("assemble_scans"))
 , twist_client   (n.serviceClient<cut_mission::GetCurrTwist>("getCurrentTwist"))
 , arrive_client  (n.serviceClient<cut_mission::CheckArrival>("checkArrival"))
 , waypoints   (new cut_mission::WaypointPairLabeled())
 , projector   (new laser_geometry::LaserProjection())
 , tf_listener (new tf::TransformListener())
 , filename  ("")
 , frame_id  ("None")
 , label     ("scan")
 , is_running(false) {

   // Ensure filename parameter is properly loaded
   n.getParam("/scan_behavior/file_name", filename);
   n.getParam("/scan_behavior/datatype", datatype);

   if (filename == "") {
     ROS_ERROR("ScanBehavior: cannot find /scan_behavivor/file_name parameter - did you run the launch file?");
     ros::shutdown();
   }
   file = std::ofstream();
   ros::service::waitForService("assemble_scans");
   ros::service::waitForService("getCurrentTwist");
   ros::service::waitForService("checkArrival");

 }

void ScanBehavior::scanCB(const sensor_msgs::LaserScan& msg) {
  // If behavior is running,
  // Transformations done referencing https://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
  // https://wiki.ros.org/tf/Tutorials/Using%20Stamped%20datatypes%20with%20tf%3A%3AMessageFilter


  if (is_running) {
    ROS_INFO("%s - recieving scan", label.c_str());
    sensor_msgs::PointCloud cloud;
    try {
      tf_listener->waitForTransform("/hokuyo", "/odom", ros::Time::now(), ros::Duration(0.05));
      projector->transformLaserScanToPointCloud("odom", msg, cloud, *tf_listener);
    } catch (tf::TransformException& e) {
        std::cout<<e.what()<<std::endl;
        return;
    }
    auto point_num = cloud.points.size();
    for (int i = 0; i<point_num; i++) {
      file<<cloud.points[i].x<<"|"<<cloud.points[i].y<<"|"<<cloud.points[i].z;
      if (i < point_num - 1) file<<",";
    }
    file<<std::endl;
    return;
  }
  return;
  }

void ScanBehavior::waypointPairCB(const cut_mission::WaypointPairLabeled& msg) {
  // If msg has behavior label, start the behavior
  if (label == msg.waypoint1.behavior.data) {
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
  waypoints->waypoint1 = p.waypoint1;
  waypoints->waypoint2 = p.waypoint2;

  // Open filename param if no file given
  if (f=="") {
    file.open(filename);
    ROS_WARN("%s - no filename given, using /scan_behavior/file_name parameter", label.c_str());
    ROS_INFO("%s - opening file for logging:\n%s", label.c_str(), filename.c_str());
  }
  else {
    file.open(f);
    ROS_INFO("%s - opening file for logging:\n%s",label.c_str(), f.c_str());
  }

  is_running = true;
  buildcloud_srv.request.begin = ros::Time::now();
  return 0;
}

void ScanBehavior::spin() {
  // Keeps node updating at rate attr. frequency

  while (n.ok()) {

    // If the behavior is currently executing
    if (is_running) {

      // Call Pathing node srv to see if arrived at wp2
      arrive_srv.request.waypoint1 = waypoints->waypoint1;
      arrive_srv.request.waypoint2 = waypoints->waypoint2;
      arrive_client.call(arrive_srv);

      if (arrive_srv.response.arrived.data == true) runHalt();  // If arrived at 2nd wp, stop
      else {
        // Get twist msg from Pathing node, label, and publish
        twist_client.call(twist_srv);
        auto msg = state_controller::TwistLabeled();
        msg.label.data = label;
        msg.twist = twist_srv.response.vector;
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
  buildcloud_srv.request.end = ros::Time::now();
  file.close();

  if (assemble_client.call(buildcloud_srv))
    printf("Got cloud with %u points\n", int(buildcloud_srv.response.cloud.points.size()));
  else
    printf("Service call failed\n");
  return 0;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ScanBehavior");
  ScanBehavior scan;
  scan.spin();
}
