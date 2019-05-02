/*
Navigates between waypoints and relies on saved hitch positions to determine next best hitch command

TODO: Make this work with platform
TODO: Add in waypoint navigation

To Run:
roslaunch tractor_sim_gazebo bringup.launch
roslaunch state_controller mainstate.launch
rosrun gravl ConvertToAckermann.py
rosrun cut_mission CutPlanner.py
rosrun cut_mission CutBehavior

*/



#include <string>
#include "CutBehavior.h"
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

CutBehavior::CutBehavior()
 : rate(ros::Rate(15))
 , hitch_pose_sub(n.subscribe("/hitch_pose", 1, &CutBehavior::CutBehavior::hitchCB, this))
 , hitch_path_sub(n.subscribe("/hitch_path", 1, &CutBehavior::CutBehavior::pathCB, this)) 
{}


void CutBehavior::spin() {
  while (n.ok()) {
    std::cout<<hitch_pose.position.x<<std::endl;

    // Check if there's a hitch path available TODO: make this based on whether behavior is active
    if (running) {
      std::cout<<"Nearest Height:"<<std::endl;
      std::cout<<getNearestHeight()<<std::endl;
    }


    ros::spinOnce();  // Update callbacks
    rate.sleep();     // Standardize output rate
  }
}

void CutBehavior::hitchCB(const geometry_msgs::Pose& msg) {
   hitch_pose = msg;
}

void CutBehavior::pathCB(const nav_msgs::Path& msg) {
  // Hitch path callback function - creates kd tree to optimize for nearest neighbor search

  hitch_path = msg;
  int n = msg.poses.size();

  // Make array from hitch path message
  kd_node_t * arr;
  arr = pathToArray(msg);

  // Create kd tree using formatted array and keep track of the root node
  KdTree new_kd;
  root = new_kd.make_tree(arr, n, 0, 2);
  kd = &new_kd;

  running = true;
}

struct kd_node_t * CutBehavior::pathToArray(nav_msgs::Path path) {
  // Convert from nav_msgs::Path message to formtted array
  // Format of output array: {{{x1,y1}, hitch_value1}, {{x2,y2}, hitch_value2}, ...}

  int num_pts = path.poses.size();
  struct kd_node_t *arr = new struct kd_node_t[num_pts];

  for (int i=0; i<num_pts; i++) {
     arr[i] = {{path.poses[i].pose.position.x, path.poses[i].pose.position.y},
                path.poses[i].pose.position.z};
  }

  return arr;
}

double CutBehavior::getNearestHeight() {
  struct kd_node_t position_node = {{hitch_pose.position.x, hitch_pose.position.y},
                                     hitch_pose.position.z};
  struct kd_node_t *nearest_node;
  double best_dist;
  kd->visited = 0;
  nearest_node = 0;

  // Find nearest node to current position
  kd->nearest(root, &position_node, 0, 2, &nearest_node, &best_dist);

  // Return height stored in nearest node
  return nearest_node->value;
}

int main(int argc, char** argv) {
   ros::init(argc, argv, "CutBehavior");
   CutBehavior cut;
   cut.spin();
}


/*
CURRENT STATUS
just use (Pose) gazebo/tractor_sim_hitch for hitch pose at any given moment
use class vairable for pose - eventually use tf to update it instead of looking it up directly

eventually will need to build in transforms between tractor and hitch

for physical platform -
  see http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29 for writing tf publisher for main chassis
  manually use hitch commands to publish hitch tf (can use same code, just use different frame id)






*/









//   tf::Transform map_to_tractor;
//   tf::poseMsgToTF(p->pose.pose, tractor_pose);
//
//
// tf::StampedTransform tractor_to_hitch;
// listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), tractor_to_hitch);
//
// tf::Transform map_to_hitch;
// map_to_hitch = map_to_tractor * tractor_to_hitch;
//
// tf::poseTFToMsg(map_to_hitch, res.pose);



//TODO: Rename this to something more general
// , pose_sub(n.subscribe("/odometry/filtered", 1, &CutBehavior::CutBehavior::odomCB, this))


// void CutBehavior::odomCB(const nav_msgs::Odometry& msg) {
//   tractor_odom = msg;
// }

//   // std::cout<<tractor_odom.pose.pose.position.x<<std::endl;
//
//
//
//   tf::StampedTransform transform;
//
//   try {
//     tf_listener.waitForTransform("chassis", "base_link",
//                               ros::Time(0), ros::Duration(3.0));
//     tf_listener.lookupTransform("chassis", "base_link",
//                              ros::Time(0), transform);
//   }    catch ( ros::Exception &e )
//     {
//      // ROS_ERROR("Error occured: %s ", e.what());
//     }





/*
subscribe to general topics
curr_hitch_cmd = 0;

call rosservice to get blade path info

make kd tree using blade path info

use kd tree to look up nearest point



get current blade pose (x, y)
  current blade pose = current tractor pose


find nearest blade pose command (x, y)

compare desired z with current z
change = difference between Z * k (tune constant)

curr_hitch_cmd += change;

get current tractor pose (x,y,z)

*/
