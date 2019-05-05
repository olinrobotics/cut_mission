#!/usr/bin/env python

import rospy
import numpy as np
import math
import helper_functions as hf
import re
import csv
import tf

# Import ROS msgs
from cut_mission.srv import CutPlan
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

class CutPlanner():

    def __init__(self):
        # Initialize ros constructs and attributes, start service

        rospy.init_node('cut_planner')
        self.service = rospy.Service('cutPlan', CutPlan, self.cutplan_call)
        self.path_pub = rospy.Publisher('/hitch_path', Path, queue_size=0)
        self.name = "cutplan"
        self.frame = 'map'
        self.file_location = None
        self.blade_width = 1.0 #meters
        self.max_cut = 0.1 #meters

        self.data_raw = [None]
        self.data_filtered = [None]
        self.data_cut = [None]
        self.path_blade = Path()
        self.path_baselink = Path()

        return

    def cutplan_call(self, req):
        # Run plan cut on data in given file, return cut path
        #TODO implement properly
        rospy.loginfo("%s - request for cut plan, %s", self.name, req)
        self.clean_attributes()
        self.plan_cut(req.filepath)
        if not self.path_blade.header.frame_id == '':
            rospy.loginfo("%s - successfully generated cut plan", self.name)
            return self.path_blade
        else:
            rospy.logerr("%s - failed to generate cut plan", self.name)
            return Path()

    def load_data(self, file):
        """ @brief load localized scan data from file
            @param[in] file: filepath to data file from scan
            @param[out]: initialize raw_data attr
            return: success boolean
            """
        try:
            with open(file, 'r') as csvfile:
                reader = csv.reader(csvfile, delimiter=",", quotechar="|")
                string_data = [[string.split('|') for string in row] for row in reader] # Massive parsing step
                self.data_raw = np.asarray(string_data, dtype=np.float64, order='C')
                rospy.loginfo("%s - finished loading raw data", self.name)
                return 0
        except IOError as e:
            rospy.logerr("%s - %s", self.name, e)
            return 1

    def filter_data(self):
        """ @brief filter scan data in class attribute
            @param[out] filtered data attr
            @return success boolean
            """

        # TODO actually filter image
        if len(self.data_raw) == 0:
            rospy.logerr("%s - filtering step cannot find raw data", self.name)
            return 1
        rospy.loginfo("%s - filtering scan & position data", self.name)
        avg_plane = self.data_raw.shape[1]/2
        self.data_filtered = np.array(self.data_raw[:,avg_plane,:])
        return 0

    def plan_cutsurface(self, data_final):
        """ @brief create data set defining new cut surface
            @param[in] filtered data attr
            @arg 1D array representing final surface x-sec
            @return success boolean
            """
        if len(self.data_filtered) == 0:
            rospy.logerr("%s - cutsurface planning step cannot find filtered data", self.name)
            return 1

        rospy.loginfo("%s - calculating next cutsurface", self.name)
        # Verify input arrays are equal length
        if not len(self.data_filtered) == len(data_final):
            rospy.logerror("%s - scan length doesn't match desired surface length", self.name)
            return 2

        # Approach 1: Take max cut depth across surface up to final surface
        data_cutraw = self.data_filtered - self.max_cut
        self.data_cut = np.clip(data_cutraw, data_final, None)

    def planpath_blade(self):
        """ @brief Generates blade and base_link poses based on data_cut
            @param[out] blade_poses attr
            @param[out] baselink_poses attr
            @return success boolean
            """

        if len(self.data_cut) == 0:
            rospy.logerr("%s - cutsurface planning step cannot find cutsurface data", self.name)
            return 1

        rospy.loginfo("%s - calculating bladepath", self.name)
        path = [None] * len(self.data_cut)

        # Calculate Pose at each point in planned cut
        for i in range(0, len(self.data_cut)):
            x = float(self.data_cut[0,0])
            y = float(self.data_cut[0,1])
            z = float(self.data_cut[0,2])
            pitch = math.tan(z/x)
            yaw = math.tan(y/x)
            [qx,qy,qz,qw] = tf.transformations.quaternion_from_euler(0,pitch,yaw) # Vector pointing to next point
            path[i] = PoseStamped(header=Header(stamp=rospy.get_rostime(),
                                                frame_id=self.frame),
                                  pose=Pose(position=Point(x,y,z),
                                            orientation=Quaternion(qx,qy,qz,qw)))

        # Store values in Path
        self.path_blade.poses = path
        self.path_blade.header.frame_id = self.frame
        self.path_blade.header.stamp = rospy.get_rostime()
        self.path_pub.publish(self.path_blade)
        return 0

    def planpath_baselink(self):
        """ @brief Generates base_link ROS Path based on path_blade

            Generates ROS Path that represents base link motion to produce
            path_blade attribute. Assumes base link sits on surface with x-axis
            parallel to line between base link point and point wheelbase
            distance away on interpolated surface.

            @param[out] path_baselink attr
            @return success boolean
            """

        if self.path_blade.header.frame_id == '':
            rospy.logerr("%s - cutsurface planning step cannot find filtered data", self.name)
            return 1

        return 2

    def check_bladeposes(self, base_path, blade_path):
        """ @brief Edit blade poses to be realizeable given their attached base_link poses
            @param[in] ROS Path base_link poses
            @param[in] ROS Path blade poses
            @return ROS Path checked blade poses
            """
        rospy.loginfo("cutplan - verifying feasibility of bladebath")
        return blade_path

    def clean_attributes(self):
        self.data_raw = [None]
        self.data_filtered = [None]
        self.data_cut = [None]
        self.path_blade = Path()
        self.path_baselink = Path()
        return

    def plan_cut(self, file_path):
        """ @brief main function - generate blade path plan from dataset
            @param[in] File containing localized scans and base link poses
            @return ROS Path verified blade poses
            """

        self.load_data(file_path)
        self.filter_data()
        data_cut = np.zeros(shape=self.data_filtered.shape) - 1
        self.plan_cutsurface(data_cut)
        self.planpath_blade()
        self.planpath_baselink()
        #bladeplan = self.check_bladeposes(path, bladeplan)
        #Send bladeplan along service
        return

    def spin(self):
        # Keeps service running
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    cp = CutPlanner()
    cp.spin()
