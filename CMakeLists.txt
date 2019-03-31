cmake_minimum_required(VERSION 2.8.3)
project(cut_mission)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

################################################
# Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
add_service_files(
 FILES
 CutPlan.srv
)

# Generate messages in the 'msg' folder
add_message_files(
 FILES
 Waypoint.msg
 WaypointPairLabeled.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cut_mission
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
