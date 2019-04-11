#!/usr/bin/env python

import rospy as rp
import numpy as np
import helper_functions as hf

# Import ROS msgs
from cut_planner.srv import CutPlan
from std_msgs.msg import Header
from nav_msgs.msg import Path

class CutPlanner():

    def __init__(self):
        """@brief initialize ros constructs, start service
            """
        rp.init_node('cut_planner')
        self.service = rp.Service('plan_bladepass', CutPlan, self.handle_cut_operation)
        self.file_location = hf.get_param_safe('/scan_to_img/file_location')
        self.blade_width = 1.0 #meters
        self.max_cut = 0.1 #meters

    def handle_cut_operation(self, req):
        # Run plan cut on data in given file, return cut path
        #TODO implement properly
        print("got a request!")
        return CutPlanResponse(Path())

    def load_data(self, file):
        """ @brief load localized scan data from file
            @param[in] file: filepath to data file from scan
            @param[out]: initialize filtered data
            """
            pass

    def filter_data(self, data):
        """ @brief filter scan data in class attribute
            @param[in] data from file
            @return 1D array of filtered data
            """

        # TODO actually filter image
        pass

    def plan_cutsurface(self, data_surf, data_final):
        """ @brief create data set defining new cut surface
            @param[in] 1D array representing surface x-sec
            @param[in] 1D array representing final surface x-sec
            @return 1D array representing next cut surface x-sec
            """

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
        pass

    def check_bladeposes(self, base_path, blade_path):
        """ @brief Edit blade poses to be realizeable given their attached base_link poses
            @param[in] ROS Path base_link poses
            @param[in] ROS Path blade poses
            @return ROS Path checked blade poses
            """
        return blade_path

    def plan_cut(self, file_path, base_path):
        """ @brief main function - generate blade path plan from dataset
            @param[in]
            @param[in] ROS Path base_link poses
            @return ROS Path verified blade poses
            """

        self.load_data(file_path)
        img_filt = filter_img(self.scan_data)
        cutsurface = plan_cutsurface(img_filt)
        bladeplan = plan_bladeposes(cutsurface, path)
        bladeplan = check_bladeposes(path, bladeplan)
        #Send bladeplan along service
        return blade_path

    def test_func(self):
        print("I'm running!")
        return

if __name__ == "__main__":
    rp.set_param('/scan_to_img/image_width', 100)
    rp.set_param('/scan_to_img/scan_size', 720)
    cp = CutPlanner()
    cp.test_func()
