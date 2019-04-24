#!/usr/bin/env python

from math import sqrt
import rospy as rp
import numpy as np
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import PointCloud2, Image
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

def nparray_to_rosimg(self, array):
    """ @brief converts numpy 2D array to ROS sensor_msgs/Image msg
        @param[in] 2D numpy array of integers
        return ROS Image
    """

    # Get min-max range
    array[array > 1E308] = 0
    array[array < 0] = 0
    min = np.min(array)
    max = np.max(array)
    array = ( (array - min)/(max-min) ) * 255
    # Scale values between 0 and 255
    image = Image()
    image.encoding = "mono8"
    image.is_bigendian = 0
    image.width = array.shape[0]
    image.height = array.shape[1]
    image.step = array.shape[0]
    image.data = [int(i) for i in array.flatten('F')]
    return image

def rosimg_to_nparray(self, image):
    """ @brief converts ROS sensor_msgs/Image msg to numpy 2D array
        @param[in] ROS Image
        return 2D numpy array of integers
    """

    array = np.zeros(image.height, image.width)
    index = 0

    for i in range(0,image.height):
        for j in range(0,image.width):
            array[i] = image.data[index]
            index += 1
    return array
