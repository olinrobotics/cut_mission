#!/usr/bin/env python

import rospy as rp
import helper_functions as hf
from cut_planner.srv import CutPlan
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from nav_msgs.msg import Path

class CutPlanner():

    def __init__(self):
        """
        @brief initialize ros constructs, start service
        """
        rp.init_node('cut_planner')
        self.service = rp.Service('plan_bladepass', CutPlan, self.handle_cut_operation)
        self.imwidth = hf.get_param_safe('/scan_to_img/image_width')
        self.imheight = hf.get_param_safe('/scan_to_img/scan_size')

    def handle_cut_operation(self, req):
        print("got a request!")
        return CutPlanResponse(Path())

    def filter_img(self, img):
        """ @brief filter scan image
            @param[in] ROS Image raw scan
            @return ROS Image filtered scan
        """
        return img

    def plan_cutsurface(self, img_scan, img_cut):
        """ @brief create scan image defining new cut surface
            @param[in] ROS Image scanned surface
            @param[in] ROS Image final cut surface
            @return ROS Image planned cut surface
            """
        return img

    def plan_bladeposes(self, img, path):
        """ @brief Generates blade poses on planned cut surface for each base_link pose
            @param[in] ROS Image planned cut surface
            @param[in] ROS Path base_link poses
            @return ROS Path blade poses
            """
        pass

    def check_bladeposes(self, base_link, blade):
        """ @brief Edit blade poses to be realizeable given their attached base_link poses
            @param[in] ROS Path base_link poses
            @param[in] ROS Path blade poses
            @return ROS Path checked blade poses
            """
        pass

    def plan_cut(self, img, path):
        """ @brief generate blade path plan from scan image
            @param[in] ROS Image raw scan
            @param[in] ROS Path base_link poses
            @param[out] ROS Path blade poses pub to service
            """

        img_filt = filter_img(img)
        cutsurface = plan_cutsurface(img_filt)
        bladeplan = plan_bladeposes(cutsurface, path)
        bladeplan = check_bladeposes(path, bladeplan)
        #Send bladeplan along service
        return

if __name__ == "__main__":
    cp = CutPlanner()
    cp.plan_cut()
