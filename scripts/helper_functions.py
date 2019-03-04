#!/usr/bin/env python

from math import sqrt
import rospy as rp
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud2
from shape_msgs.msg import Plane

def get_avgaltitude_sweep(pose, scan, thresh=0.1):
    """
    @brief Gives avg z-val of all scan points in plane normal to pose vector
    @param[in] ros pose msg
    @param[in] ros pointcloud msg
    @param[out] ros point msg
    """

    # Get plane with pose normal vector and pose point
    norm_vec = pose.orientation * Vector3(1,0,0)
    sweep_pts = []
    for point in scan.data:
        if dist_to_plane(point, plane) < thresh:
            sweep_pts.append(point)

    #return average of sweep_point z-values
    return

def dist_to_plane(point, plane):
    """
    @brief Gives distance from point to plane
    @param[in] ros point msg
    @param[in] ros plane msg
    @return shortest dist
    """
    return abs(plane.coef[0]*point.x + plane.coef[1]*point.y + plane.coef[2]*point.z + plane.coef[3])/(sqrt(plane.coef[0]**2 + plane.coef[1]**2 + plane.coef[2]**2))

def get_param_safe(param):
    """
    @brief Checks for param existence, error and shut down node if dne
    @param[in] ros param string
    @return value of param, None if no param
    """

    if not rp.has_param(param):
        rp.signal_shutdown("See ros error log")
        rp.logerr("Cannot get param " + param + " - Did you init this parameter correctly?")
        return None

    return rp.get_param(param)
