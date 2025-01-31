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
        self.flag_line_tracer = True
        self.flag_driving_mode = 1
        # self.flag_traffic_light = True
        # self.flag_dynamic = True
        # self.flag_static = True
        self.flag_tunnel = False
        self.flag_labacorn = False
        # self.flag_stopline = True
        self.flag_parking_mode = True
        self.flag_parking_current_state = 0
        self.flag_parking_cal_step = 0
        self.count = 0

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
        self.Lidar_range_change(100, 260, 0.6)
        self.left_labacorn_deg = []
        self.left_labacorn_dist = []
        self.right_labacorn_deg = []
        self.right_labacorn_dist = []

        for idx, dist in enumerate(self.laba_objects):

            dist_deg = dist.laba_deg
            dist_dist = dist.laba_dist
            first_deg = dist_deg[0]

            if first_deg > self.lidar_index_half:
                self.left_labacorn_deg.append(dist_deg)
                self.left_labacorn_dist.append(dist_dist)
            elif first_deg < self.lidar_index_half:
                self.right_labacorn_deg.append(dist_deg)
                self.right_labacorn_dist.append(dist_dist)
            if len(self.left_labacorn_deg) >= 1 and len(self.right_labacorn_deg) >= 1:
                self.left_pair = [[x[0], y[0]] for x, y in zip(self.left_labacorn_deg, self.left_labacorn_dist)]
                self.left_close_dist = sorted(self.left_pair, key=lambda x: x[1])[0][1]
                self.right_pair = [[x[0], y[0]] for x, y in zip(self.right_labacorn_deg, self.right_labacorn_dist)]
                self.right_close_dist = sorted(self.right_pair, key=lambda x: x[1])[0][1]
                self.laba = (self.left_close_dist + self.left_close_dist) / 2
                

    #####################   Labacorn Drive   #####################
    def Labacorn_tracer(self, speed=0):    
        if self.laba:
            steering = -(self.right_close_dist - self.left_close_dist) * 1.4
        else:
            steering = 0

        # print(steering)
        if steering > 0.34:
            steering = 0.34
        elif steering < -0.34:
            steering = -0.34
        

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


    #####################   Avoid   #####################
    def Inline_static_avoid(self):
        print("avoid inline")
        self.Lidar_range_change(170, 190, 0.8)
        start_time = rospy.get_time()
        end_time = rospy.get_time()

        while end_time - start_time <= 1.2:
            self.Drive_controller(0.3,-3)
            end_time = rospy.get_time()
        while end_time - start_time <= 2:
            self.Drive_controller(0.3,3)
            end_time = rospy.get_time()
        
        self.flag_driving_mode = 4
        self.flag_static = True
        self.flag_labacorn = False

    def Outline_static_avoid(self):
        print("avoid outline")
        self.Lidar_range_change(170, 190, 0.8)
        start_time = rospy.get_time()
        end_time = rospy.get_time()

        while end_time - start_time <= 1.2:
            self.Drive_controller(0.3,3)
            end_time = rospy.get_time()
        while end_time - start_time <= 2:
            self.Drive_controller(0.3,-3)
            end_time = rospy.get_time()

        # self.flag_driving_mode = 4
        # self.flag_static = True
        # self.flag_labacorn = False


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

    
    def Lidar_detect(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        self.Lidar_range_change(170, 190, 00.6)

        self.Drive_stop()


    #####################   Parking Hard Coding   #####################
    def Parking_hard_detect(self):
        ## Hard Coding Lidar Change
        self.Lidar_range_change(265,275,0.65)

        for idx, val in enumerate(self.parking_objects):
            self.parking_dist = float(val.laba_dist[0])
            self.parking_deg = int(val.laba_deg[0])
            # print(self.parking_dist)
            
        if self.parking_dist is None:
            self.parking_dist = self.parking_dist
            return
        
        if self.parking_lidar_previous_distance is None:
            self.parking_lidar_previous_distance = self.parking_dist
            return

        if self.flag_parking_current_state == 0:
            if self.parking_dist - self.parking_lidar_previous_distance >= 0.3:
                print(f"state=0 : {self.parking_dist - self.parking_lidar_previous_distance}")
                self.flag_parking_current_state = 1
        
        elif self.flag_parking_current_state == 1:
            if self.parking_dist - self.parking_lidar_previous_distance <= 0.1:
                print(f"state=1 : {self.parking_dist - self.parking_lidar_previous_distance}")
                self.flag_parking_current_state = 2
        
        elif self.flag_parking_current_state == 2 and self.num_object == 0:
            self.flag_parking_current_state = 3


        # print(self.parking_dist)
        # print(self.parking_deg)
        # print(f"{self.parking_dist} - {self.parking_lidar_previous_distance}")
        # print(self.parking_dist_mean)
        print(self.flag_parking_current_state)

    def Parking_hard_0(self):
        start_time = rospy.get_time()
        end_time = rospy.get_time()
        while end_time - start_time <= 1.95:
            self.Drive_controller(0.2,0.0)
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

        self.flag_parking_mode = False

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

        self.flag_line_tracer = True
    
    def Parking_hard(self):
        self.Drive_stop()
        self.Parking_hard_0()
        self.Drive_stop()
        self.Parking_hard_1()
        self.Drive_stop()
        self.Parking_hard_2()
        self.Drive_stop()
        self.Getout_hard_1()
        self.Getout_hard_2()


    #####################   Parking Calculate Coding   #####################
    def Parking_cal_detect(self):
        self.Lidar_range_change(85,95,0.7)
        parking_deg_list = []
        parking_dist_list = []
        parking_dist = None
        parking_dist = None


        for idx, val in enumerate(self.parking_objects):
            self.parking_dist = float(val.laba_dist[0])
            self.parking_deg = int(val.laba_deg[0])
            # print(self.parking_dist)
            
        if self.parking_dist is None:
            self.parking_dist = self.parking_dist
            return
        
        if self.parking_lidar_previous_distance is None:
            self.parking_lidar_previous_distance = self.parking_dist
            return

        if self.flag_parking_current_state == 0:
            if self.parking_dist - self.parking_lidar_previous_distance >= 0.23:
                # print(f"state=1 : {self.parking_dist - self.parking_lidar_previous_distance}")
                self.flag_parking_current_state = 1
        
        elif self.flag_parking_current_state == 1:
            if self.parking_dist - self.parking_lidar_previous_distance <= 0.1:
                # print(f"state=2 : {self.parking_dist - self.parking_lidar_previous_distance}")
                self.flag_parking_current_state = 2

        elif self.flag_parking_current_state == 2 :
            self.Lidar_range_change(1, 40, 0.4)
            if 120 <= self.parking_deg <= 135 or 0.33 <= self.parking_dist <= 0.35:
                parking_deg_list.append(self.parking_deg)
                parking_dist_list.append(self.parking_dist)
                self.flag_parking_current_state = 3
            
        elif self.flag_parking_current_state == 3:
            self.Drive_stop()
            self.flag_parking_current_state = 4

        # print(f"dist:{self.parking_dist},  deg:{self.parking_deg}")
        print(self.flag_parking_current_state)

    def Parking_cal_1(self):
        # print("parking_cal_1 start")
        self.Lidar_range_change(1, 120, 0.42)
        parking_cal_list_1 = []
        for idx, val in enumerate(self.parking_objects):
            parking_deg = int(val.laba_deg[0])
            if 400 < parking_deg < 420 :
                parking_cal_list_1.append(parking_deg) 
        if len(parking_cal_list_1) == 0:
            self.Drive_controller(-0.2,-3.4)
        else:
            self.Drive_controller(0,0)
            # self.Lidar_range_change(310,359,0.7)
            self.flag_parking_cal_step = 1
        print(f"Parking_cal_1:{parking_cal_list_1}")

    def Parking_cal_2(self):
        self.Lidar_range_change(310,359,0.7)
        # self.Drive_stop()
        print("parking_cal_2 start")
        for idx, val in enumerate(self.parking_objects):
            parking_dist = float(val.laba_dist[0])
            parking_deg = int(val.laba_deg[0])
            if 1100 < parking_deg <= 1110 and parking_dist < 0.33:
                self.parking_cal_list_2.append(parking_dist)
        if len(self.parking_cal_list_2) == 0:
            self.Drive_controller(-0.2, 3.4)
        else:
            self.Drive_controller(0,0)
            self.flag_parking_cal_step = 2
        print(f"Parking_cal_2:{self.parking_cal_list_2}")
        
    def Getout_cal_1(self):
        print("Getout_cal_1 start")
        self.Lidar_range_change(120,140,0.35)
        getout_cal_list = []
        for idx, val in enumerate(self.parking_objects):
            parking_deg = int(val.laba_deg[0])
            if 430 < parking_deg < 500:
                getout_cal_list.append(parking_deg)
        print(getout_cal_list)
        if len(getout_cal_list) == 0:
            self.Drive_controller(0.2, 3.4)
        else:
            self.Drive_controller(0,0)
            self.flag_parking_cal_step = 4

    def Getout_cal_2(self):
        print("Getout_cal_2 start")
        self.Lidar_range_change(25,40,0.4)
        getout_cal_list = []
        for idx, val in enumerate(self.parking_objects):
            parking_deg = int(val.laba_deg[0])
            if 140 <= parking_deg < 160:
                getout_cal_list.append(parking_deg)
        print(getout_cal_list)
        if len(getout_cal_list) == 0:
            self.Drive_controller(0.2, -2.4)
        else:
            # self.Drive_controller(0,0)
            self.flag_parking_cal_step = 5
            self.flag_line_tracer = True
            self.flag_parking_mode = False

    def Parking_cal(self):
        if self.flag_parking_current_state == 4 and self.flag_parking_cal_step == 0:
            self.Parking_cal_1()
        elif self.flag_parking_cal_step == 1:
            self.Parking_cal_2()
        elif self.flag_parking_cal_step == 2:
            self.Drive_stop()
            self.flag_parking_cal_step = 3
        elif self.flag_parking_cal_step == 3:
            self.Getout_cal_1()
        elif self.flag_parking_cal_step == 4:
            self.Getout_cal_2()
        elif self.flag_parking_cal_step == 5 and self.flag_line_tracer == True:
            self.Line_tracer(self.input_speed)

        print(self.flag_parking_cal_step)


    def Traffic_Color_Detect(self):
        pass


    def Roundabout(self):
        pass


    def Drive_tunnel(self):
        print("tunnel drive")
        self.Lidar_range_change(130, 230, 0.55)
        self.left_tunnel_deg = []
        self.left_tunnel_dist = []
        self.right_tunnel_deg = []
        self.right_tunnel_dist = []

        for idx, dist in enumerate(self.laba_objects):

            dist_deg = dist.laba_deg
            dist_dist = dist.laba_dist
            first_deg = dist_deg[0]

            if first_deg > self.lidar_index_half:
                self.left_tunnel_deg.append(dist_deg)
                self.left_tunnel_dist.append(dist_dist)
            elif first_deg < self.lidar_index_half:
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

    def Traffic_Line_Change(self):
        pass

    #####################   Action   #####################
    def Timer_CB(self, event):
        # ## Parking_Hard
        # self.Color_judge()
        # if self.flag_line_tracer == True:
        #     self.Line_tracer(self.input_speed)
        # if self.light_color == "Blue":
        #     while self.flag_parking_mode == True:
        #         self.flag_line_tracer = False
        #         self.Line_tracer(0.2)
        #         self.Parking_hard_detect()
        #         if self.flag_parking_current_state == 3:
        #             self.Drive_stop()
        #             if self.flag_parking_mode == True:
        #                 self.Parking_hard_0()
        #                 self.Drive_stop()
        #                 self.Parking_hard_1()
        #                 self.Drive_stop()
        #                 self.Parking_hard_2()
        #                 # self.Drive_stop()
        #                 # self.Getout_hard_1()
        #                 # self.Drive_stop()
        #                 # self.Getout_hard_2()
        self.Parking_hard()

        
        # self.Parking_hard_detect()


if __name__ == "__main__":
    try:
        cl = Controller()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except IndexError:
        print("list index out of range")