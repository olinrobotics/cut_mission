#!/usr/bin/env python

import rospy as rp
import numpy as np
import helper_functions as hf
from sensor_msgs.msg import LaserScan, PointCloud2, Image  # PointCloud2
from std_msgs.msg import Header, Float32        # Header

class LaserToImage():

    def __init__(self):
        """
        @brief initialize ros constructs
        """
        rp.init_node('scan_to_img')
        self.temp = 0
        self.use_pc = hf.get_param_safe('/scan_to_img/use_pc')
        self.imwidth = hf.get_param_safe('/scan_to_img/image_width')
        self.imheight = hf.get_param_safe('/scan_to_img/scan_size')

        if not self.use_pc:
            self.laser_sub = rp.Subscriber('/scan', LaserScan, self.scan_cb)
            rp.loginfo("Using input \"sensor_msgs/LaserScan\"")
            self.scan_count = 0
        else:
            self.laser_sub = rp.Subscriber('/laser_pointcloud', PointCloud2, self.pc_cb)
            rp.loginfo("Using intput \"sensor_msgs/PointCloud2\"")

        self.image_sub = rp.Subscriber('/usb_cam/image_raw', Image, self.im_cb)
        self.image_pub = rp.Publisher('/image_scan', Image, queue_size=1)

        self.data_matrix = np.zeros((self.imwidth, self.imheight))
        self.rate = rp.Rate(1)

    def pc_cb(self, msg):
        pass

    def im_cb(self, msg):
        if self.temp == 0:
            print(msg.height)
            print(msg.width)
            print(msg.encoding)
            print(msg.is_bigendian)
            print(msg.step)
            self.temp = 1

    def scan_cb(self, msg):

        self.data_matrix[self.scan_count,:] = msg.ranges
        self.scan_count += 1
        if self.scan_count == self.imwidth:
            self.image_pub.publish(self.nparray_to_rosimg(self.data_matrix))
            self.scan_count = 0

    def nparray_to_rosimg(self, array):
        """ @brief converts numpy 2D array to ROS image
            @param[in] 2D numpy array of integers
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

    def run(self):
        while not rp.is_shutdown():
            rp.spin()

if __name__ == '__main__':
    inst = LaserToImage()
    inst.run()
