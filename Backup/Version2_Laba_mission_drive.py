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
        # self.flag_traffic_light = True
        # self.flag_dynamic = True
        # self.flag_static = True
        # self.flag_labacorn = True
        # self.flag_stopline = True
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
        

    #####################   Labacorn Point Detect   #####################
    def Drive_labacorn(self):
        print("Labacorn drive")
        self.flag_labacorn = True
        self.Lidar_range_change(120, 240, 0.8)
        self.left_labacorn_deg = []
        self.left_labacorn_dist = []
        self.right_labacorn_deg = []
        self.right_labacorn_dist = []

        self.left_close_dist = 0
        self.right_close_dist = 0

        for idx, dist in enumerate(self.objects):

            dist_deg = dist.deg
            dist_dist = dist.dist
            left_first_deg = dist_deg[0]
            right_first_deg = dist_deg[1]
            print(f"left_first_deg:{left_first_deg}, right_first_deg:{right_first_deg}")
            if left_first_deg > self.lidar_index_half:
                self.left_labacorn_deg.append(dist_deg)
                self.left_labacorn_dist.append(dist_dist)
            elif right_first_deg < self.lidar_index_half:
                self.right_labacorn_deg.append(dist_deg)
                self.right_labacorn_dist.append(dist_dist)
            if len(self.left_labacorn_deg) >= 1 and len(self.right_labacorn_deg) >= 1:
                self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_labacorn_deg, self.left_labacorn_dist)]
                self.left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
                self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_labacorn_deg, self.right_labacorn_dist)]
                self.right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
                self.laba = (self.left_close_dist + self.left_close_dist) / 2
                
            # print(f"left_list:{self.left_labacorn_deg}")
            # print(f"right_list:{self.right_labacorn_deg}")
        print(f"right_dist:{self.right_close_dist}, left_dist:{self.left_close_dist}")

    #####################   Labacorn Drive   #####################
    def Labacorn_tracer(self, speed=0):    
        if self.laba:
            steering = -(self.right_close_dist - self.left_close_dist) * 1.3
        else:
            steering = 0

        # print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        
        # self.flag_labacorn = True

        self.Drive_controller(speed, steering)

    
    #####################   Color Judge   #####################
    def Color_judge(self):
        if self.light_color == 'Blue':
            self.light_color = 'Blue'


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
        if self.num_object == 1:
            self.Drive_stop()
            if self.num_object and self.flag_static == False:
                # self.flag_driving_mode = 2
                pass
            elif self.num_object and self.flag_static == True and self.flag_dynamic == False:
                self.Drive_controller(0,0)
                if self.num_object == 0:
                    self.flag_dynamic = True
                    # self.flag_driving_mode = 0
                    # self.flag_labacorn == False


    #####################   Action   #####################
    def Timer_CB(self, event):
        ### 라바콘 -> 차로 색상 인식(노랑) -> 회전교차로(회전 주행 차량 회피) -> 횡단보도(카메라)
        ### -> 터널 -> 차선 변경(마커 인식) -> 차단기 -> 주차(표지판 인식 후 주차)

        ### 현재 가능한 것
        ### : 라바콘, 횡단보도, 차단기, 주차(칸막이가 있다면 표지판 인식 후 주차까지 가능)
        
        ### 해야 하는 것
        ### : 차로 색상 빨간색 추가, 회전교차로, 터널(라이다 주행이랑 비슷할 것이라 생각)
        ### : 카메라만 사용해서 주차, 차선 변경(마커 인식)


        ### 메커니즘 if문
        ### 상세한 것은 추가적으로 더 작성해야 함.
        # if self.flag_driving_mode == 0:
        #     self.Line_tracer(self.input_speed)
        #     if self.num_object > 2:
        #         self.flag_driving_mode = 2
        
        # elif self.flag_driving_mode == 1:
        #     self.Labacorn_tracer(0.3)
        # elif self.flag_driving_mode == 2:
        #     self.Traffic_Color_Detect()
        #     # 노란색이므로 그대로 색 그대로 가져와서 사용하면 됨.
        # elif self.flag_driving_mode == 3:
        #     self.Roundabout()
        # elif self.flag_driving_mode == 4:
        #     # Stopline
        #     self.Line_tracer()
        #     if self.stopline_check == True:
        #         self.Drive_stop()
        #         # 약 3~4초 기다렸다가 출발 or 신호등
        # elif self.flag_driving_mode == 5:
        #     self.Tunnel()
        # elif self.flag_driving_mode == 6:
        #     self.Traffic_Line_Change()
        # elif self.flag_driving_mode == 7:
        #     pass
        #     # 차단기 -> 라이다로 멈췄다가 그대로 이동(동적 장애물)
        # elif self.flag_driving_mode == 8:
        #     self.Color_judge()
        #     if  self.light_color == "Blue":
        #         while self.flag_parking_mode == True:
        #             self.flag_line_tracer = False
        #             self.Line_tracer(0.2)
        #             self.Parking_hard_detect()
        #             if self.flag_parking_current_state == 4:
        #                 self.Drive_stop()
        #                 if self.flag_parking_mode == True:
        #                     self.Parking_hard()

        self.Drive_labacorn()
        self.Labacorn_tracer(0.33)
            
        

if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")