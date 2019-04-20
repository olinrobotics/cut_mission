#!/usr/bin/env python

import rospy
import numpy
import helper_functions as hf

# Import ROS msgs
from cut_mission.srv import CutPlan
from std_msgs.msg import Header
from nav_msgs.msg import Path

class CutPlanner():

    def __init__(self):
        """@brief initialize ros constructs, start service
            """
        rospy.init_node('cut_planner')
        self.service = rospy.Service('plan_bladepass', CutPlan, self.handle_cut_operation)
        self.file_location = hf.get_param_safe('/scan_to_img/file_location')
        self.blade_width = 1.0 #meters
        self.max_cut = 0.1 #meters

    def handle_cut_operation(self, req):
        # Run plan cut on data in given file, return cut path
        #TODO implement properly
        rospy.loginfo("cutplan - request for cut plan given file %s", req)
        self.plan_cut(req)
        return CutPlanResponse(Path())

    def load_data(self, file):
        """ @brief load localized scan data from file
            @param[in] file: filepath to data file from scan
            @param[out]: initialize filtered data
            """
        rospy.loginfo("cutplan - loading data from file %s", file)
        pass

    def filter_data(self, data):
        """ @brief filter scan data in class attribute
            @param[in] data from file
            @return 1D array of filtered data
            """

        # TODO actually filter image
        rospy.loginfo("cutplan - filtering scan & position data")
        pass

    def plan_cutsurface(self, data_surf, data_final):
        """ @brief create data set defining new cut surface
            @param[in] 1D array representing surface x-sec
            @param[in] 1D array representing final surface x-sec
            @return 1D array representing next cut surface x-sec
            """
        rospy.loginfo("cutplan - calculating next cutsurface")
        # Verify input arrays are equal length
        if not len(data_surf) == len(data_final):
            rospy.logerror("plan_cutsurface(): scan length doesn't match planned cut length")
            return
        data_cut = [None] * len(data_surf)

        # Approach 1: Take max cut depth across surface up to final surface
        for i in range(len(data_surf)):
            cut_depth_left = data_surf[i] - data_final[i]
            if cut_depth_left <= self.max_cut: # If
                data_cut[i] = cut_depth_left
            else:
                data_cut[i] = data_surf[i] - self.max_cut
        return data_cut

    def plan_bladeposes(self, data_cut, base_path):
        """ @brief Generates blade poses on planned cut surface for each base_link pose
            @param[in] 1D array representing next cut surface x-sec
            @param[in] ROS Path base_link poses
            @return ROS Path blade poses
            """

        rospy.loginfo("cutplan - calculating bladepath for cutsurface")
        pass

    def check_bladeposes(self, base_path, blade_path):
        """ @brief Edit blade poses to be realizeable given their attached base_link poses
            @param[in] ROS Path base_link poses
            @param[in] ROS Path blade poses
            @return ROS Path checked blade poses
            """
        rospy.loginfo("cutplan - verifying feasibility of bladebath")
        return blade_path

    def plan_cut(self, file_path):
        """ @brief main function - generate blade path plan from dataset
            @param[in] File containing localized scans and base link poses
            @return ROS Path verified blade poses
            """

        self.load_data(file_path)
        img_filt = self.filter_data(self.scan_data)
        cutsurface = self.plan_cutsurface(img_filt)
        bladeplan = self.plan_bladeposes(cutsurface, path)
        bladeplan = self.check_bladeposes(path, bladeplan)
        #Send bladeplan along service
        return blade_path

    def spin(self):
        # Keeps service running
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    rospy.set_param('/scan_to_img/file_location', "$(find cut_mission)/config/test.txt")
    cp = CutPlanner()
    cp.spin()
