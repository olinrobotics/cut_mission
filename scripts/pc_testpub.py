#!/usr/bin/env python

import rospy as rp
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from random import random


class TestDataPublisher():

    def __init__(self, topic, node):
        self.pc_pub = rp.Publisher(topic, PointCloud2, queue_size=1)
        rp.init_node(node)
        self.pointcloud = PointCloud2()
        cloud = self.create_pc_plane(10.0,200,10.0, 200, 1.0, True)
        self.pointcloud.header = Header(stamp=rp.get_rostime(), frame_id='main')
        self.pointcloud = pc2.create_cloud_xyz32(self.pointcloud.header, cloud)

    def send_scan(self):
        self.pc_pub.publish(self.pointcloud)

    def create_pc_plane(self, x_dist, x_num, y_dist, y_num, z, add_noise=False):
        pc = []
        for i in range(0,x_num):
            for j in range(0, y_num):
                coeff = 0.0
                if add_noise: coeff = 0.1
                pc.append([i * x_dist/(x_num-1) + coeff*random(), j * y_dist/(y_num-1) + coeff*random(), z + coeff*random()])
        return pc

    def run(self):
        while not rp.is_shutdown():
            self.send_scan()
            rp.sleep(1.0)


if __name__ == "__main__":
    tp = TestDataPublisher('roadscan', 'testscan_gen')
    tp.run()
