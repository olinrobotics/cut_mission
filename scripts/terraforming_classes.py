#!/usr/bin/env python
import rospy as rp
import helper_functions as hf
from sensor_msgs.msg import PointCloud2, Image         # PointCloud2 - curr. road surface
from geometry_msgs.msg import PoseArray, Polygon, Point# PoseArray - pose list of curr. road, Polygon - vis of Plane
from shape_msgs.msg import Plane                # Plane - goal road
from std_msgs.msg import Header, Float32        # Header - stamps, float - cut depth



class CutPlanner():

    def __init__(self, rviz_show=False):
        """
        @brief init node, pubs, subs, params
        """

        rp.init_node("cut_planner")
        self.scan_sub = rp.Subscriber('roadscan', PointCloud2, self.roadscan_cb)
        self.pose_sub = rp.Subscriber('roadpose', PoseArray, self.roadposes_cb)
        #TODO: Init bladeposearray pub

        # TODO: set params in launch file
        rp.set_param('/goalplane/a', 0.0)
        rp.set_param('/goalplane/b', 0.0)
        rp.set_param('/goalplane/c', 0.0)
        rp.set_param('/goalplane/d', 3.0)
        rp.set_param('/maxdepth_cut', 0.5)

        self.z_max = hf.get_param_safe('/maxdepth_cut')
        self.goalplane = Plane(hf.get_param_safe('/goalplane'))

        if rviz_show: self.vis = CutVisualizer()

        # Initialize vars for incoming data storage
        self.roadposes = None
        self.roadscan = None
        self.roadchunks = []
        self.posechunks = []
        self.roadheight_avg = []

        pass

    def roadscan_cb(self, msg):
        """
        @brief callback func for road surface scan pointcloud
        @param[in] pointcloud2 from subscriber
        @param[out] roadscan attr
        """
        self.roadscan = msg
        if hasattr(self, 'vis'):
            self.vis.show_roadscan(msg)

    def roadposes_cb(self, msg):
        """
        @brief callback func for posearray defining road
        @param[in] posearray from subscriber
        @param[out] roadscan attr
        """
        self.roadposes = msg
        if hasattr(self, 'vis'):
            self.vis.show_roadposes(msg)

    def check_for_scan(self):
        """
        @brief waits for init data to be populated, then returns
        @return 1 if rp shutdown, 0 if success
        """

        while not rp.is_shutdown():

            case = 2 * (self.roadposes == None) + (self.roadscan == None)
            if case == 0:
                rp.loginfo("Recieved all necessary data - beginning cut path planning")
                return 0
            elif case == 1:
                rp.loginfo_throttle(10, "Recieved road poses")
                rp.loginfo_throttle(1, "Cutplanner node waiting for road scan")
            elif case == 2:
                rp.loginfo_throttle(10, "Recieved road scan")
                rp.loginfo_throttle(1, "Cutplanner node waiting for road poses")
            elif case == 3:
                rp.loginfo_throttle(1, "Cutplanner node waiting for road poses and scans")
                rp.sleep(1.0)

        return 1

    def map_avg_altitude(self):
        """
        @brief creates 2D map of road based on scan data
        """

        # Calls get_avg_altitude for all poses in roadposes attr
        for pose in self.roadposes.poses:
            self.roadheight_avg.append(self.getavgaltitude_sweep(pose))

    def plan_cutpass(self, point_list):

        """
        @brief creates cut passes until scan equal to planned road surface
        """
        pass

    def save_cutplan(self):
        """
        @brief saves and/or publishes current cutplan
        """
        pass

    def run(self):
        """
        @brief plans cut sequence for given data & road plan; saves sequence
        """

        if not self.check_for_scan():
            #TODO: possibly section scan?
            self.map_avg_altitude()
            self.create_cutplan()
            self.save_cutplan()
            #TODO: reset?
            return

class CutVisualizer():

    def __init__(self):
        """
        @brief init pubs, subs
        """

        self.scan_pub = rp.Publisher('/vis/roadscan', PointCloud2, queue_size=1)
        self.pose_pub = rp.Publisher('/vis/roadposes', PoseArray, queue_size=1)
        self.plane_pub = rp.Publisher('vis/goalplane', Polygon, queue_size=1)
        self.cut_pub = rp.Publisher('/vis/cutsurfaces', Polygon, queue_size=1)

    def show_roadscan(self, pc):
        """
        @brief displays pointcloud in rviz
        @param[in] Ros pointcloud2 msg
        """
        self.scan_pub.publish(pc)

    def show_roadposes(self, posearray):
        """
        @brief displays pointcloud in rviz
        @param[in] Ros PoseArray msg
        """
        self.pose_pub.publish(posearray)

if __name__ == "__main__":
    planner = CutPlanner(True)
    pt = Point(x=1.0,y=-5.0,z=3.0)
    pln = Plane([4,0,1,-5])
    print(hf.getavgaltitude_sweep())
    #planner.run()
