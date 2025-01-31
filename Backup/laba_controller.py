#!/usr/bin/env python3

import rospy
import cv2
from smarp_msgs.msg import objectStatus, objInfo, lidarStatus, camInfo
from ackermann_msgs.msg import AckermannDriveStamped
import time
import math

class Controller :
    def __init__(self):
        rospy.init_node("Controller")

        ##################  Lidar  ##################
        self.lidar_index_half_list = [643,643]
        self.lidar_index_half = 643

        self.lidar_status_msg = lidarStatus()
        self.num_object = 0
        self.laba = 0
        

        self.steer_denominator = 300

        ##################  Race  ##################
        self.drive_data = AckermannDriveStamped()
        rospy.Subscriber("/lidar_data", objectStatus, self.Lidar_CB)
        self.lidar_range_change_pub = rospy.Publisher("/lidar_change_range", lidarStatus, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/10), self.Timer_CB)      

        self.drive_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.drive_data = AckermannDriveStamped()

    ##################   Lidar_Camrea Callback   ##################
    def Lidar_CB(self, msg):
        
        self.num_object = msg.no_objects
        self.objects = msg.objects
        self.div_deg = list(msg.deg)
        self.lidar_dist = list(msg.dist)

        # print(my_pair)

        # print(combined_list)

        
                

        # print(self.left_labacorn_deg)
        
        # print(self.right_close_list)
    #####################   Labacorn Drive   #####################
    def Drive_labacorn(self):
        self.left_labacorn_deg = []
        self.left_labacorn_dist = []
        self.right_labacorn_deg = []
        self.right_labacorn_dist = []

        for idx, dist in enumerate(self.objects):

            dist_deg = dist.deg
            dist_dist = dist.dist
            first_deg = dist_deg[0]

            if first_deg > self.lidar_index_half:
                self.left_labacorn_deg.append(dist_deg)
                self.left_labacorn_dist.append(dist_dist)
            elif first_deg < self.lidar_index_half:
                self.right_labacorn_deg.append(dist_deg)
                self.right_labacorn_dist.append(dist_dist)
                # print(self.left_labacorn_deg)
            if len(self.left_labacorn_deg) >= 1 and len(self.right_labacorn_deg) >= 1:
                # left_pair = list(zip(self.div_deg, self.lidar_dist))
                self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_labacorn_deg, self.left_labacorn_dist)]
                self.left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
                self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_labacorn_deg, self.right_labacorn_dist)]
                self.right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
                # print(self.left_pair)
                # print(self.right_close_dist)
                self.laba = (self.left_close_dist + self.left_close_dist) / 2
                # print(self.laba)

#####################   Speed Control   #####################
    def Drive_controller(self, speed, steer):
        self.drive_data.drive.speed = speed
        self.drive_data.drive.steering_angle = steer
        self.drive_pub.publish(self.drive_data)

    def Laba_tracer(self, speed):

        if self.laba:
            steering = -(self.right_close_dist - self.left_close_dist)
        else:
            steering = 0

        print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        
        self.Drive_controller(speed, steering)

    #####################   Action   #####################
    def Timer_CB(self, event):
        self.Drive_labacorn()
        self.Laba_tracer(0.2)
        pass


if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")