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
        self.input_speed = 0.3
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
        # self.flag_traffic_light = True
        # self.flag_labacorn = True
        self.flag_parking_mode = True
        self.flag_parking_current_state = 0
        self.flag_parking_cal_step = 0
        self.static_flag = False
        self.dynamic_flag = False
        self.mark_flag = False
        self.stopline_check = False
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
        # print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 3.0:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()

    #####################   Color Judge   #####################
    # def Color_judge(self):
    #     if self.light_color == 'Blue':
    #         self.light_color = 'Blue'


    ############## MISSION FUNCTION ##############
    def Drive_labacorn(self):
        self.flag_line_tracer = False
        if self.num_object == 0:
            self.flag_driving_mode = 2
        # print("Labacorn drive")
        self.flag_labacorn = True
        # self.Lidar_range_change(120, 240, 0.8)
        self.Lidar_range_change(140, 220, 1.0)

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
            # print(f"left_first_deg:{left_first_deg}, right_first_deg:{right_first_deg}")
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
        # print(f"right_dist:{self.right_close_dist}, left_dist:{self.left_close_dist}")

                

    def Labacorn_tracer(self, speed=0):    
        if self.laba:
            steering = -(self.right_close_dist - self.left_close_dist) * 1.5
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
        print(f"redzone {self.redzone_detect}")
        if self.redzone_detect == True:
            while self.redzone_detect == True:
                self.Line_tracer(0.22)
            self.flag_driving_mode = 3


    def Stopline_stop(self):
        # print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 6.0:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()    
        self.flag_driving_mode = 4
        
    def Obj_to_tun(self):
        # print("Drive_stop")
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 0.7:
            self.Drive_controller(0.2,0.2)
            end_time = rospy.get_time()

    # def Roundabout(self):
    #     self.Lidar_range_change(150,240,0.6)
    #     self.Line_tracer(self.input_speed)
    #     if self.num_object > 0:
    #         self.Drive_stop()
    #         self.flag_driving_mode = 5
    #     else:
    #         self.flag_driving_mode = 5
    
    # def Rot_Object_detect(self):
    #     self.Lidar_range_change(150,240,0.6)
    #     # self.Line_tracer(self.input_speed)
    #     # self.Drive_stop()
    #     while self.num_object == 1:
    #         print("Rot Object Detect")
    #         self.Drive_stop()
    #     self.flag_driving_mode = 5


    # def Object_detect(self):
    #     self.Lidar_range_change(170,190,0.6)
    #     print("Object Detect")
    #     self.Drive_stop()
    #     while self.num_object == 1:
    #         self.Drive_stop()
    #     self.flag_driving_mode = 6


    # def Tunnel_detect(self):
    #     self.Lidar_range_change(130,230,0.50)



    def Drive_tunnel(self, speed = 0):
        print("tunnel point detect")
        # self.Lidar_range_change(130,230,0.50)
        # self.left_tunnel_deg = []
        # self.left_tunnel_dist = []
        # self.right_tunnel_deg = []
        # self.right_tunnel_dist = []

        # self.tunnel_left_close_dist = 0
        # self.tunnel_right_close_dist = 0


        # for idx, dist in enumerate(self.objects):

        #     dist_deg = dist.deg
        #     dist_dist = dist.dist
        #     left_first_deg = dist_deg[0]
        #     right_first_deg = dist_deg[1]
        #     # print(f"left_first_deg:{left_first_deg}, right_first_deg:{right_first_deg}")
        #     if left_first_deg > self.lidar_index_half:
        #         self.left_tunnel_deg.append(dist_deg)
        #         self.left_tunnel_dist.append(dist_dist)
        #     if right_first_deg < self.lidar_index_half:
        #         self.right_tunnel_deg.append(dist_deg)
        #         self.right_tunnel_dist.append(dist_dist)
        #     if len(self.left_tunnel_deg) >= 1 and len(self.right_tunnel_deg) >= 1:
        #         self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
        #         self.tunnel_left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
        #         self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
        #         self.tunnel_right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
        #         self.laba = (self.right_tunnel_dist[0][0] + self.left_tunnel_dist[0][0]) / 2
        #         # print(self.laba)
        #     print(f"left:{self.tunnel_left_close_dist}, right{self.tunnel_right_close_dist}")


        # self.Lidar_range_change(89,181,0.5)
        
        self.Lidar_range_change(110,181,0.5)

        self.left_labacorn_deg = []
        self.left_labacorn_dist = []
        self.right_labacorn_deg = []
        self.right_labacorn_dist = []
        ddd = 0.29
        steering = 0
        self.left_close_dist = 0
        self.right_close_dist = 0

        for idx, dist in enumerate(self.objects):

            dist_deg = dist.deg
            dist_dist = dist.dist
            left_first_deg = dist_deg[0]
            right_first_deg = dist_deg[1]
            # print(f"left_first_deg:{left_first_deg}, right_first_deg:{right_first_deg}")
            # if left_first_deg > self.lidar_index_half:
            #     self.left_labacorn_deg.append(dist_deg)
            #     self.left_labacorn_dist.append(dist_dist)
            if right_first_deg < self.lidar_index_half:
                self.right_labacorn_deg.append(dist_deg)
                self.right_labacorn_dist.append(dist_dist)
            if len(self.right_labacorn_deg) >= 1:
                # self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_labacorn_deg, self.left_labacorn_dist)]
                # self.left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
                self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_labacorn_deg, self.right_labacorn_dist)]
                self.right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
                # self.laba = (self.left_close_dist + self.left_close_dist) / 2
                # print(self.right_close_dist)
                if self.right_close_dist > 0:
                    steering = -(self.right_close_dist - ddd) * 2.5
                    # print(steering)
                    self.Drive_controller(speed, steering)

                else :
                    steering = (self.right_close_dist - ddd) *2

                # print(steering)
                if steering > 0.34:
                    steering = 0.34
                elif steering < -0.34:
                    steering = -0.34
                # print(f"left_list:{self.left_labacorn_deg}")
            # print(f"right_list:{self.right_labacorn_deg}")
        # print(f"right_dist:{self.right_close_dist}, left_dist:{self.left_close_dist}")






        # self.left_tunnel_deg = []
        # self.left_tunnel_dist = []
        # self.right_tunnel_deg = []
        # self.right_tunnel_dist = []
        # steering = 0
        # self.tunnel_left_close_dist = 0
        # self.tunnel_right_close_dist = 0

        # ddd = 0.28

        # for idx, dist in enumerate(self.objects):
        #     self.left_tunnel_deg = []
        #     self.left_tunnel_dist = []
        #     self.right_tunnel_deg = []
        #     self.right_tunnel_dist = []
        #     dist_deg = dist.deg
        #     dist_dist = dist.dist
        #     print(dist_dist)
        #     left_first_deg = dist_deg[0]
        #     right_first_deg = dist_deg[1]
        #     # print(f"left_first_deg:{left_first_deg}, right_first_deg:{right_first_deg}")
        #     # if left_first_deg > self.lidar_index_half:
        #     #     self.left_tunnel_deg.append(dist_deg)
        #     #     self.left_tunnel_dist.append(dist_dist)
        #     if right_first_deg < self.lidar_index_half:
        #         self.right_tunnel_deg.append(dist_deg)
        #         self.right_tunnel_dist.append(dist_dist)
        #     # if len(self.left_tunnel_deg) >= 1 and len(self.right_tunnel_deg) >= 1:
        #     #     self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
        #     #     self.tunnel_left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
        #     #     self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
        #     #     self.tunnel_right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
        #     #     self.laba = (self.right_tunnel_dist[0][0] + self.left_tunnel_dist[0][0]) / 2
        #         # print(self.laba)
        #     # print(f"left:{self.tunnel_left_close_dist}, right{self.tunnel_right_close_dist}")

        #     if len(self.right_tunnel_deg) >= 1:
        #         self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
        #         self.tunnel_right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
        #         # print(self.tunnel_right_close_dist)
        # print(f"right_dist:{self.tunnel_right_close_dist}")
        
        # # left_first_deg = self.objects[0].deg[0]
            # print(self.tunnel_right_close_dist)

        # try:
        #     left_first_deg = self.objects[0].deg[0]
        #     left_first_dist = self.objects[0].dist[0]
        #     right_first_deg = self.objects[0].deg[1]
        #     right_first_dist = self.objects[0].dist[1]

        #     if right_first_deg < self.lidar_index_half:
        #         self.right_tunnel_deg.append(right_first_deg)
        #         self.right_tunnel_dist.append(right_first_dist)
        #     if left_first_deg > self.lidar_index_half:
        #         self.left_tunnel_deg.append(left_first_deg)
        #         self.left_tunnel_dist.append(left_first_dist)

        #     if len(self.left_tunnel_deg) >= 1 and len(self.right_tunnel_deg) >= 1:
        #         self.tunnel_left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
        #         self.tunnel_left_close_dist = sorted(self.tunnel_left_pair, key=lambda x: x[1])[0][1]
        #         self.tunnel_right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
        #         self.tunnel_right_close_dist = sorted(self.tunnel_right_pair, key=lambda x: x[1])[0][1]
        #         self.laba = (self.tunnel_left_close_dist + self.tunnel_left_close_dist) / 2

        #     print(f"left_first_dist:{self.left_tunnel_deg},   right_first_dist:{self.right_tunnel_deg}")
        # except IndexError:
        #     pass

        # for idx, dist in enumerate(self.objects):

        #     dist_deg = dist.deg
        #     dist_dist = dist.dist
        #     left_first_deg = dist_deg[0]
        #     right_first_deg = dist_deg[1]

        #     if left_first_deg > self.lidar_index_half:
        #         self.left_tunnel_deg.append(dist_deg)
        #         self.left_tunnel_dist.append(dist_dist)
        #     elif right_first_deg < self.lidar_index_half:
        #         self.right_tunnel_deg.append(dist_deg)
        #         self.right_tunnel_dist.append(dist_dist)
        #     if len(self.left_tunnel_deg) >= 1 and len(self.right_tunnel_deg) >= 1:
        #         self.tunnel_left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
        #         self.tunnel_left_close_dist = sorted(self.tunnel_left_pair, key=lambda x: x[1])[0][1]
        #         self.tunnel_right_pair = [[x[0], y[0]] for x, y in zip(self.right_tunnel_deg, self.right_tunnel_dist)]
        #         self.tunnel_right_close_dist = sorted(self.tunnel_right_pair, key=lambda x: x[1])[0][1]
        #         self.laba = (self.tunnel_left_close_dist + self.tunnel_left_close_dist) / 2


        # for idx, dist in enumerate(self.objects):

        #     dist_deg = dist.deg
        #     dist_dist = dist.dist
        #     left_first_deg = dist_deg[0]
        #     right_first_deg = dist_deg[1]

        #     if left_first_deg > self.lidar_index_half:
        #         self.left_tunnel_deg.append(dist_deg)
        #         self.left_tunnel_dist.append(dist_dist)
        #     if len(self.left_tunnel_deg) >= 1:
        #         self.tunnel_left_pair = [[x[0], y[0]] for x, y in zip(self.left_tunnel_deg, self.left_tunnel_dist)]
        #         self.tunnel_left_close_dist = sorted(self.tunnel_left_pair, key=lambda x: x[1])[0][1]
        # print(f"self.left_tunnel_deg:{self.left_tunnel_deg},   right_trunnel_deg:{self.left_tunnel_deg}")
        # print(self.right_tunnel_deg)
        # print(f"tunnel_left_close_dist:{self.tunnel_left_close_dist},  turnnel_right_close_dist:{self.tunnel_right_close_dist}")


    def Tunnel(self, speed=0):
        print("tunnel drive")
        # steering = 0
        # steering = -(self.tunnel_right_close_dist - self.tunnel_left_close_dist) - 0.15
        # ddd = 0.42
        # left_error = self.tunnel_left_close_dist - ddd
        # right_error = self.tunnel_right_close_dist - ddd
        # steering = -(right_error - left_error)


        # if self.tunnel_left_close_dist > ddd:
        #     steering = self.tunnel_left_close_dist - ddd * 1.5
        # elif self.tunnel_left_close_dist < ddd:
        #     steering = -(self.tunnel_left_close_dist - ddd) * 1.5

        # print(f"steering:{steering} = tunnel_left_clost_dist{self.tunnel_left_close_dist} - ddd{ddd}")
        # # print(steering)

        # steering = 0
        # desired_distance = 0.6
        # if self.tunnel_left_close_dist < desired_distance : #왼쪽 비례 조향 조정

        #     left_error = self.tunnel_left_close_dist - desired_distance

            # steering = left_error #왼쪽 비례 조향....
        # elif self.tunnel_left_close_dist > desired_distance:
        #     left_error = (self.tunnel_left_close_dist - desired_distance)
        #     steering = -left_error  #왼쪽 비례 조향....
        # if self.tunnel_left_close_dist:
        #     left_error = self.tunnel_left_close_dist - desired_distance
        #     steering = left_error * 5 #왼쪽 비례 조향....
                
        
        # print(steering)
        steering = 0
        ddd = 0.3
        
        # steering = ((self.tunnel_right_close_dist - ddd)) / 3

            # if self.num_object == 0:
            #     print("none")
            #     break
        # else:
        #     print("none")
        #     steering = 0


        if self.right_close_dist:
            steering = (self.right_close_dist - ddd) * 1.5
            print(steering)
        # print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        
        self.Drive_controller(speed, steering)


    def Line_Change_Value(self):
        # print(self.ar_mark)
        try:
            for idx, val in enumerate(self.ar_mark):
                self.mark_id = val.id
                # print(self.mark_id)
                        
                ### 약 50cm -> 0.35
                self.mark_dist.append(val.pose.pose.position.z)
                # print(self.mark_dist)

                if len(self.mark_dist) > 0:
                    if self.mark_dist[-1] < 0.35:
                        self.Drive_stop()
                        self.mark_flag = True
        except AttributeError:
            pass
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
            # print(f"pre_obs_deg:{pre_obs_deg}, cur_obs_deg:{cur_obs_deg}")

        except IndexError:
            pass

    
     #####################   Parking Hard Coding   #####################
    # def Parking_hard_detect(self):
    #     ## Hard Coding Lidar Change
    #     self.Lidar_range_change(265,275,0.65)

    #     for idx, val in enumerate(self.parking_objects):
    #         self.parking_dist = float(val.laba_dist[0])
    #         self.parking_deg = int(val.laba_deg[0])
    #         # print(self.parking_dist)
            
    #     if self.parking_dist is None:
    #         self.parking_dist = self.parking_dist
    #         return
        
    #     if self.parking_lidar_previous_distance is None:
    #         self.parking_lidar_previous_distance = self.parking_dist
    #         return

    #     if self.flag_parking_current_state == 0:
    #         if self.parking_dist - self.parking_lidar_previous_distance >= 0.3:
    #             print(f"state=0 : {self.parking_dist - self.parking_lidar_previous_distance}")
    #             self.flag_parking_current_state = 1
        
    #     elif self.flag_parking_current_state == 1:
    #         if self.parking_dist - self.parking_lidar_previous_distance <= 0.1:
    #             print(f"state=1 : {self.parking_dist - self.parking_lidar_previous_distance}")
    #             self.flag_parking_current_state = 2
        
    #     elif self.flag_parking_current_state == 2 and self.num_object == 0:
    #         self.flag_parking_current_state = 3


    #     # print(self.parking_dist)
    #     # print(self.parking_deg)
    #     # print(f"{self.parking_dist} - {self.parking_lidar_previous_distance}")
    #     # print(self.parking_dist_mean)
    #     print(self.flag_parking_current_state)

    def T_hard_0(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 1.3:
            self.Drive_controller(0.2,0.1)
            end_time = rospy.get_time()


    def Parking_hard_0(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 3.2:
            self.Drive_controller(0.2,0)
            end_time = rospy.get_time()


    ### Speed = -0.2 Steering = -3.4
    def Parking_hard_1(self):
        self.Lidar_range_change(1,35,0.75)
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 2.68:
            self.Drive_controller(-0.2,3.4)
            end_time = rospy.get_time()

    def Parking_hard_2(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 2.1:
            self.Drive_controller(-0.2,-2.9)
            end_time = rospy.get_time()

        # self.flag_parking_mode = False

    def Parking_stop(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 6.5:
            self.Drive_controller(0,0)
            end_time = rospy.get_time()

    def Getout_hard_0(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 0.8:
            self.Drive_controller(-0.2,0)
            end_time = rospy.get_time()
    
    def Getout_hard_1(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 2.0:
            self.Drive_controller(0.2,-3.4)
            end_time = rospy.get_time()
    
    def Getout_hard_2(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 2.6:
            self.Drive_controller(0.2,3.4)
            end_time = rospy.get_time()

        # self.flag_line_tracer = True
    
    def Parking_hard(self):
        print("parking")
        self.Drive_stop()
        self.Parking_hard_0()
        self.Drive_stop()
        self.Parking_hard_1()
        self.Drive_stop()
        self.Parking_hard_2()
        self.Parking_stop()
        self.Getout_hard_0()
        self.Drive_stop()
        self.Getout_hard_1()
        self.Drive_stop()
        self.Getout_hard_2()

    

    #####################   Action   #####################
    def Timer_CB(self, event):
        ## 라바콘 -> 차로 색상 인식(노랑) -> 회전교차로(회전 주행 차량 회피) -> 횡단보도(카메라)
        ## -> 터널 -> 차선 변경(마커 인식) -> 차단기 -> 주차(표지판 인식 후 주차)
        rospy.loginfo(f"self.flag_driving_mode = {self.flag_driving_mode}")
        
        if self.flag_line_tracer == True: #라인 트레이싱
            self.Line_tracer(self.input_speed)

        if self.flag_driving_mode == 1 and self.num_object >= 2: #라바콘 주행
            self.flag_line_tracer = False
            self.Drive_labacorn()
            self.Labacorn_tracer(0.20)
            if self.num_object == 0:
                self.flag_driving_mode = 2

        if self.num_object == 0 and self.flag_line_tracer == False and self.flag_driving_mode == 1:
            self.Flag_up()

        if self.flag_driving_mode == 2: #어린이 보호구역
            self.Redzone()

        if self.flag_driving_mode == 3: #횡단보도
            if self.stopline_check == True:
                self.Stopline_stop()
                pass
        
        if self.flag_driving_mode == 4: #로터리,차단기
            self.Lidar_range_change(130, 230, 0.6)
            if self.num_object == True:
                while True:
                    if self.num_object == False:
                        break
                    self.Drive_controller(0,0)
                    self.Object_type_decision()
                    
            if self.static_flag == True:
                self.flag_driving_mode = 5

        if self.flag_driving_mode == 5 and self.static_flag == True and self.num_object >= 1: #터널
            while True:                
                self.Drive_tunnel(0.20)
                self.Line_Change_Value()
                if self.mark_flag == True :
                    break
            self.Traffic_Line_Change()
            self.flag_driving_mode = 6
        
        if self.flag_driving_mode == 6 and self.stopline_check == True: #주차
            self.flag_line_tracer = False
            self.Parking_hard()
        # print(f"mode{self.flag_driving_mode}")
        # print(f"line{self.flag_line_tracer}")
        # print(f"static_flag:{self.static_flag}")
        # print(f"num_object:{self.num_object}")
if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")