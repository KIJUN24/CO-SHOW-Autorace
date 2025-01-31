#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from smarp_msgs.msg import objectStatus, objInfo, lidarStatus
from math import *


class LidarScan :
    def __init__(self):
        rospy.init_node("Lidar_scan")
        self.lidar_dist = 0.8
        self.lidar_range = [393,894]
        self.lidar_range_half = 643
        self.grouping_size_limit = 0.1
        self.grouping_min_deg = 10
        self.lidar_parking_check = False

        # self.scan_ranges = []
        # self.length_of_ranges = 1285
        # self.half_of_ranges = self.length_of_ranges // 2
        # self.start_roi = 170
        # self.end_roi = 190
        # self.start_index = int((self.lidar_start_roi/360) * self.length_of_ranges)
        # self.end_index = int((self.end_roi/360)*self.length_of_ranges)
        # self.roi_distance = 1.0

        rospy.Subscriber("/scan", LaserScan, self.Lidar_callback)
        self.lidar_pub = rospy.Publisher("lidar_data", objectStatus, queue_size=5)
        self.objects_pub = objectStatus()


    def Lidar_callback(self, data) :
        self.scan_msg = data
        # print(len(self.scan_msg.ranges))
        obstacles = self.Lidar_filter(self.lidar_range[0], self.lidar_range[1], self.lidar_dist)
        self.objects_pub = self.Lidar_grouping(obstacles, self.grouping_size_limit)
        self.lidar_pub.publish(self.objects_pub)



    def Lidar_filter(self, right_angle, left_angle, distance):
        self.scan_msg.ranges = self.scan_msg.ranges[self.lidar_range_half:] + self.scan_msg.ranges[:self.lidar_range_half]
        obstacles = []
        for index in range(right_angle, left_angle + 1):
            if self.scan_msg.ranges[index] < distance:
                obstacles.append([index, self.scan_msg.ranges[index]])
        # print(obstacles)
        return obstacles
    

    def Lidar_grouping(self, obstacles, size_limit):
        no_group = 0
        pre_obj_deg = 0
        pre_obj_dist = 0
        one_grp = objInfo()
        obs_info = objectStatus()

        for idx, [deg, dist] in enumerate(obstacles):
            if (pre_obj_deg == 0):
                one_grp.deg.append(deg)
                one_grp.dist.append(dist)
                pre_obj_deg = deg
                pre_obj_dist = dist
                no_group += 1
            elif (idx == len(obstacles) - 1):
                one_grp.deg.append(pre_obj_deg)
                one_grp.dist.append(pre_obj_dist)
                obs_info.objects.append(one_grp)
                # obs_info.deg.append(pre_obj_deg)
                # obs_info.dist.append(pre_obj_dist)
            elif ((deg - pre_obj_deg) > self.grouping_min_deg or abs(pre_obj_dist - dist) > size_limit):
                # one_grp.deg.append(pre_obj_deg)
                # one_grp.dist.append(pre_obj_dist)
                obs_info.objects.append(one_grp)
                one_grp = objInfo()
                pre_obj_deg = deg
                pre_obj_dist = dist
                no_group += 1
                one_grp.deg.append(deg)
                one_grp.dist.append(dist)
            else:
                pre_obj_deg = deg
                pre_obj_dist = dist

        obs_info.no_objects = no_group
        # print(obs_info)
        return obs_info


if __name__ == "__main__":
    try:
        ls = LidarScan()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    # except IndexError:
    #     pass