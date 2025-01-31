#!/usr/bin/env python3

import rospy
import cv2
from smarp_msgs.msg import objectStatus, objInfo, lidarStatus, camInfo, laba_objInfo
from ackermann_msgs.msg import AckermannDriveStamped
import time
import math

class Controller :
    def __init__(self):
        rospy.init_node("Controller")
        
        self.start_time = rospy.get_time()

        ##################  Line  ##################
        self.follow_line = 'L'
        self.lx = 115
        self.rx = 520
        self.labacorn_count = 0

        ##################  Varible  ##################
        # self.lx_reference_value = 110
        # self.rx_reference_value = 515
        self.lx_reference_value = 100
        self.rx_reference_value = 555
        self.steer_denominator = 460
        # speed : -250 ~ 250
        self.input_speed = 0.53

        ##################  Lidar  ##################
        self.lidar_status_msg = lidarStatus()
        self.lidar_index_half = 643
        self.num_object = 0
        self.lidar_const = 3.5722222
        self.lidar_change_left_angle = 0
        self.lidar_change_right_angle = 100
        self.lidar_change_distance = 0.7
        self.labacorn_left_point = 0
        self.labacorn_right_point = 0
        self.laba = 0
        self.parking_cal_list_2 = []
        self.laba_objects = None
        self.parking_point_count = 0
        self.parking_lidar_previous_distance = None
        self.parking_dist_mean = None
        self.parking_dist = None

        ##################  Flag  ##################
        self.flag_line_tracer = False
        self.flag_driving_mode = 0
        self.flag_static = True
        self.flag_parking_mode = True
        self.flag_parking_current_state = 0
        self.flag_parking_cal_step = 0

        ##################  Race  ##################
        self.drive_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.drive_data = AckermannDriveStamped()
        rospy.Subscriber("/line_data", camInfo, self.Cam_sub_CB)
        rospy.Subscriber("/lidar_data", objectStatus, self.Lidar_CB)
        self.lidar_range_change_pub = rospy.Publisher("/lidar_change_range", lidarStatus, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/10), self.Timer_CB)      



    ##################   Lidar_Camrea Callback   ##################
    def Lidar_CB(self, msg):
        self.num_object = msg.no_objects
        self.objects = msg.objects
        self.laba_objects = msg.laba_objects
        self.parking_objects = msg.laba_objects

    def Cam_sub_CB(self, msg):
        self.lx = msg.line_point[0]
        self.rx = msg.line_point[1]
        self.light_color = msg.light_color
        self.stopline_check = msg.stopline_check
        # print(f"left:{self.lx},   right:{self.rx},   light_color:{self.light_color}")


    #####################   Lidar Change   #####################
    def Lidar_range_change(self, lower_angle, upper_angle, distance):
        right_index = math.floor(lower_angle * self.lidar_const)
        left_index = math.ceil(upper_angle * self.lidar_const)
        if left_index > 1285:
            left_index = 1285
        # print(f"right_index:{right_index},  left_index:{left_index}")
        self.lidar_status_msg.range = (right_index, left_index)
        self.lidar_status_msg.dist = distance
        self.lidar_range_change_pub.publish(self.lidar_status_msg)


    #####################   Speed Control   #####################
    def Drive_controller(self, speed, steer):
        self.drive_data.drive.speed = speed
        self.drive_data.drive.steering_angle = steer
        self.drive_pub.publish(self.drive_data)


    #####################   Line Drive   #####################
    def Line_tracer(self, speed=0):
        print("line drive")
        if self.lx <= 0:
            self.follow_line = 'R'
        elif self.rx >= 640:
            self.follow_line = 'L'

        if self.follow_line == 'R':
            steering = (self.rx_reference_value - self.rx) / self.steer_denominator
            # print(f"RIGHT DETECT : lx={self.lx}, rx={self.rx}, speed={speed}, steering={steering}")
        elif self.follow_line == 'L':
            steering = (self.lx_reference_value - self.lx) / self.steer_denominator
            # print(f"LEFT DETECT : lx={self.lx}, rx={self.rx}, speed={speed}, steering={steering}")
        else:
            steering = 0

        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        self.Drive_controller(speed, steering)


    #####################   Mission Stop   #####################
    def Drive_stop(self):
        print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 3.0:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()


    #####################   Object Detect   #####################
    def Object_detect(self):
        self.Lidar_range_change(170,190,0.6)
        print("Object Detect")
        self.Drive_stop()

        if self.num_object == 1:
            self.Drive_stop()
            if self.num_object >= 1:
                # self.flag_driving_mode = 2
                print("Not yet")
                self.Drive_stop()
            else:
                print("Go")
                self.flag_static = False

    def Driving_mode_count(self):
        self.flag_driving_mode = 2
    
    #####################   Action   #####################
    def Timer_CB(self, event):
        ### 라바콘 -> 차로 색상 인식(노랑) -> 회전교차로(회전 주행 차량 회피) -> 횡단보도(카메라)
        ### -> 터널 -> 차선 변경(마커 인식) -> 차단기 -> 주차(표지판 인식 후 주차)

        ### 현재 가능한 것
        ### : 라바콘, 횡단보도, 차단기, 주차(칸막이가 있다면 표지판 인식 후 주차까지 가능)
        
        ### 해야 하는 것
        ### : 차로 색상 빨간색 추가, 회전교차로, 터널(라이다 주행이랑 비슷할 것이라 생각)
        ### : 카메라만 사용해서 주차, 차선 변경(마커 인식)

        if self.flag_driving_mode == 0:
            self.Line_tracer(self.input_speed)

        if self.num_object == 1:
            self.Driving_mode_count()

        if self.num_object == 1 and self.flag_static == True and self.flag_driving_mode == 2:
            self.Object_detect()
            
        if self.flag_driving_mode == 2:
            # next mission
            self.Line_tracer(self.input_speed)

        print(f"num_objct={self.num_object}, flag_driving_mode={self.flag_driving_mode}")        

if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")