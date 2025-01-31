#!/usr/bin/env python3

import rospy
import cv2
from smarp_msgs.msg import objectStatus, objInfo, lidarStatus, camInfo, laba_objInfo
from ackermann_msgs.msg import AckermannDriveStamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from geometry_msgs.msg import Point
import time
import math

class Controller :
    def __init__(self):
        rospy.init_node("Controller")
        
        self.start_time = rospy.get_time()

        ##################  Line  ##################
        self.follow_line = 'R'
        # self.lx = 115
        # self.rx = 520
        self.lx = 50
        self.rx = 600
        self.labacorn_count = 0

        ##################  Varible  ##################
        # self.lx_reference_value = 110
        # self.rx_reference_value = 515
        # self.lx_reference_value = 100
        # self.rx_reference_value = 555
        # self.steer_denominator = 480
        self.lx_reference_value = 30
        self.rx_reference_value = 580
        self.steer_denominator = 290
        # speed : -250 ~ 250
        self.input_speed = 0.35
        self.mark_dist = []

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
        self.flag_line_tracer = True
        self.flag_driving_mode = 1
        self.flag_static = True
        # self.flag_traffic_light = True
        # self.flag_labacorn = True
        self.flag_parking_mode = True
        self.flag_parking_current_state = 0
        self.flag_parking_cal_step = 0
        self.static_flag = False
        self.dynamic_flag = False
        self.mark_flag = False
        ##################  Race  ##################
        self.drive_pub = rospy.Publisher("/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.drive_data = AckermannDriveStamped()
        rospy.Subscriber("/line_data", camInfo, self.Cam_sub_CB)
        rospy.Subscriber("/lidar_data", objectStatus, self.Lidar_CB)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.Mark_CB)
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
        self.redzone_detect = msg.redzone_check
        # print(f"left:{self.lx},   right:{self.rx},   light_color:{self.light_color}")

    def Mark_CB(self, msg):
        self.ar_mark = msg.markers

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
        if self.lx <= 30:
            self.lx = 0
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


    def Drive_stop(self):
        print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 3.0:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()

    #####################   Color Judge   #####################
    def Color_judge(self):
        if self.light_color == 'Blue':
            self.light_color = 'Blue'


    ############## MISSION FUNCTION ##############
    def Drive_labacorn(self):
        self.flag_line_tracer = False
        if self.num_object == 0:
            self.flag_driving_mode = 2
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

                

    def Labacorn_tracer(self, speed=0):    
        if self.laba:
            steering = -((self.right_close_dist - self.left_close_dist) * 1.6) + 0.07
        else:
            steering = 0

        # print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        
        # self.flag_labacorn = True
        if self.num_object == 0:
            self.flag_driving_mode = 2

        self.Drive_controller(speed, steering)


    def Redzone(self):
        print(self.redzone_detect)
        if self.redzone_detect == True:
            while self.redzone_detect == True:
                self.Line_tracer(0.25)
            self.flag_driving_mode = 3


    def Stopline_stop(self):
        print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 6.0:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()    
        self.flag_driving_mode = 4    


    # def Roundabout(self):
    #     self.Lidar_range_change(150,240,0.6)
    #     self.Line_tracer(self.input_speed)
    #     if self.num_object > 0:
    #         self.Drive_stop()
    #         self.flag_driving_mode = 5
    #     else:
    #         self.flag_driving_mode = 5
    
    def Rot_Object_detect(self):
        self.Lidar_range_change(150,240,0.6)
        # self.Line_tracer(self.input_speed)
        # self.Drive_stop()
        while self.num_object == 1:
            print("Rot Object Detect")
            self.Drive_stop()
        self.flag_driving_mode = 5


    def Object_detect(self):
        self.Lidar_range_change(170,190,0.6)
        print("Object Detect")
        self.Drive_stop()
        while self.num_object == 1:
            self.Drive_stop()
        self.flag_driving_mode = 6



    def Drive_tunnel(self):
        print("tunnel drive")
        self.left_tunnel_deg = []
        self.left_tunnel_dist = []
        self.right_tunnel_deg = []
        self.right_tunnel_dist = []

        self.tunnel_left_close_dist = 0
        self.tunnel_right_close_dist = 0

        for idx, dist in enumerate(self.objects):

            dist_deg = dist.deg
            dist_dist = dist.dist
            left_first_deg = dist_deg[0]
            right_first_deg = dist_deg[1]

            if left_first_deg > self.lidar_index_half:
                self.left_tunnel_deg.append(dist_deg)
                self.left_tunnel_dist.append(dist_dist)
            elif right_first_deg < self.lidar_index_half:
                self.right_tunnel_deg.append(dist_deg)
                self.right_tunnel_dist.append(dist_dist)
            if len(self.left_tunnel_deg) >= 1 and len(self.right_tunnel_deg) >= 1:
                self.tunnel_left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
                self.tunnel_left_close_dist = sorted(self.tunnel_left_pair, key=lambda x: x[1])[0][1]
                self.tunnel_right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
                self.tunnel_right_close_dist = sorted(self.tunnel_right_pair, key=lambda x: x[1])[0][1]
                self.laba = (self.tunnel_left_close_dist + self.tunnel_left_close_dist) / 2
                
    def Tunnel(self, speed=0):
        if self.laba:
            steering = -(self.tunnel_right_close_dist - self.tunnel_left_close_dist) - 0.03
        else:
            steering = 0

        # print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        
        self.Drive_controller(speed, steering)


    def Line_Change_Value(self):
        # print(self.ar_mark)
        
        for idx, val in enumerate(self.ar_mark):
            self.mark_id = val.id
            # print(self.mark_id)
                       
            ### 약 50cm -> 0.35
            self.mark_dist.append(val.pose.pose.position.z)
            # print(self.mark_dist)

            if len(self.mark_dist) > 0:
                if self.mark_dist[-1] < 0.48:
                    self.Drive_stop()
                    self.mark_flag = True

    def Traffic_Line_Change(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        if(self.mark_id == 0):
            # self.follow_line = 'L'
            # self.Line_tracer(0.27)
            print("turn left")
            while end_time - start_time <= 1.8:
                self.Drive_controller(0.2,2.7)
                end_time = rospy.get_time()

        elif(self.mark_id == 4):
            # self.follow_line = 'R'
            # self.Line_tracer(0.27)
            print("turn right")
            while end_time - start_time <= 1.8:
                self.Drive_controller(0.2,-2.7)
                end_time = rospy.get_time()

    def Flag_up(self):
        self.flag_line_tracer = True
        self.flag_driving_mode = 2

    def Object_type_decision(self):
        start_time = rospy.get_time()
        end_time = 0

        try:
            pre_obs_deg = self.objects[0].deg[0]

            while end_time - start_time <= 1.0:
                cur_obs_deg = self.objects[0].deg[0]
                end_time = rospy.get_time()


            # if abs(cur_obs_deg - pre_obs_deg) >= 20:
            #     rospy.loginfo("dynamic_obs")
            #     self.dynamic_flag = True
            if abs(cur_obs_deg - pre_obs_deg) <= 10:
                rospy.loginfo("static_obs")
                self.static_flag = True
            print(f"pre_obs_deg:{pre_obs_deg}, cur_obs_deg:{cur_obs_deg}")

        except IndexError:
            pass
    

    #####################   Action   #####################
    def Timer_CB(self, event):
        ## 라바콘 -> 차로 색상 인식(노랑) -> 회전교차로(회전 주행 차량 회피) -> 횡단보도(카메라)
        ## -> 터널 -> 차선 변경(마커 인식) -> 차단기 -> 주차(표지판 인식 후 주차)
        rospy.loginfo(f"self.flag_driving_mode = {self.flag_driving_mode}")
        print(self.num_object)
        if self.flag_line_tracer == True: #라인 트레이싱
            self.Line_tracer(self.input_speed)

        if self.flag_driving_mode == 1 and self.num_object >= 2: #라바콘 주행
            self.flag_line_tracer = False
            self.Drive_labacorn()
            self.Labacorn_tracer(0.3)
            if self.num_object == 0:
                self.flag_driving_mode = 2

        if self.num_object == 0 and self.flag_line_tracer == False and self.flag_driving_mode == 1:
            self.Flag_up()

        if self.flag_driving_mode == 2: #어린이 보호구역
            self.Redzone()

        if self.flag_driving_mode == 3: #횡단보도
            if self.stopline_check == True:
                self.Stopline_stop()
                self.Lidar_range_change(140, 220, 1.6)
                pass
        
        if self.flag_driving_mode == 4: #로터리,차단기
            rospy.loginfo(f"distance = {self.objects[0].dist[0]}")
            try :
                if self.objects[0].dist[0] < 130:
                    while True:
                        if self.objects[0].dist[0] >= 130:
                            self.Lidar_range_change(150, 210, 60)
                            break
                        self.Drive_controller(0,0)

                elif self.objects[0].dist[0] >= 130:
                    self.Lidar_range_change(150, 210, 60)

                if self.num_object == True:
                    while True:
                        if self.num_object == False:
                            break
                        self.Drive_controller(0,0)
                        self.Object_type_decision()
                        if self.static_flag == True:
                            self.flag_driving_mode = 5
            
            except IndexError:
                pass

        if self.flag_driving_mode == 5 and self.num_object == 2: #터널
            while True:
                self.Drive_tunnel()
                self.Tunnel(0.3)
                self.Line_Change_Value()
                if self.mark_flag == True :
                    break
            self.Traffic_Line_Change()
        
        # if self.flag_driving_mode == 6: #A,B 차선 진입
        #     self.Traffic_Line_Change()

        print(f"mode{self.flag_driving_mode}")
        print(f"line{self.flag_line_tracer}")
        
if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")