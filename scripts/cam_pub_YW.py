#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
from smarp_msgs.msg import camInfo
import numpy as np
from math import *


class CameraRec :
    def __init__(self):
        rospy.init_node("Camera_rec")
        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.Cam_callback)
        self.bridge = CvBridge()
        # self.canny_pub = rospy.Publisher("Canny_img", CompressedImage, queue_size=5)
        self.canny_pub = rospy.Publisher("Canny_img", camInfo, queue_size=5)
        self.line_pub = rospy.Publisher("/line_data", camInfo, queue_size=10)
        self.line_data = camInfo()

        # line yellow & white
        # self.yellow_lower = np.array([15, 50, 50])
        # self.yellow_upper = np.array([60, 255, 255])

        # self.white_lower = np.array([30, 0, 60])
        # self.white_upper = np.array([60, 255, 255])



        self.yellow_lower = np.array([20, 80, 50])
        self.yellow_upper = np.array([35, 255, 255])
        self.white_lower = np.array([30, 0, 90])
        self.white_upper = np.array([255, 255, 255])

        # self.white_lower = np.array([0, 0, 120])
        # self.white_upper = np.array([255, 255, 255])


        # self.white_lower = np.array([40, 0, 170])
        # self.white_upper = np.array([170, 35, 220])

        # self.white_lower = np.array([15, 50, 60])
        # self.white_upper = np.array([80, 255, 255])

        # self.white_lower = np.array([50,0,100])
        # self.white_upper = np.array([179,25,255])

        # traffic light R, G, B, O
        self.light_color = ''
        # self.blue_lower = np.array([85, 175, 90])
        # self.blue_upper = np.array([110, 255, 255])

        self.blue_lower = np.array([100, 100, 30])
        self.blue_upper = np.array([130, 255, 100])
        
        
        
        # point reset
        self.left_point_x = 0
        self.right_point_x = 0
        self.vertical_value = 80
        # self.vertical_value = 60
        self.center_point = 320

        # roi variable
        self.roi_line_up = 300
        self.roi_line_down = 480
        self.roi_line = 300

        self.roi_traffic_light_left_x1 = 130
        self.roi_traffic_light_left_x2 = 400
        self.roi_traffic_light_left_y1 = 0
        self.roi_traffic_light_left_y2 = 180
        self.roi_traffic_light_right_x1 = 130
        self.roi_traffic_light_right_x2 = 400
        self.roi_traffic_light_right_y1 = 420
        self.roi_traffic_light_right_y2 = 640
        self.roi_stopline_x1 = 20
        self.roi_stopline_x2 = 100
        self.roi_stopline_y1 = 180
        self.roi_stopline_y2 = 420

        # stopline variable
        self.stopline_val = 5000
        self.stopline_detct = False
        
        # redzone
        self.redzone_detect = False
        self.redzone_lower = np.array([0, 100, 30])
        self.redzone_upper = np.array([10, 180, 80])  

        # self.redzone_lower = np.array([0, 50, 30])
        # self.redzone_upper = np.array([20, 255, 100])  


    # white yellow detect
    def Color_detect(self, img):
        # line detect
        rgb_img = img
        self.img_line_crop = img.copy()[self.roi_line_up: , :]
        
        hsv_line = cv2.cvtColor(self.img_line_crop, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv_line, self.yellow_lower, self.yellow_upper)

        # blend_mask = cv2.bitwise_or(yellow_mask, white_mask)
        # blend_color = cv2.bitwise_and(hsv_line, hsv_line, mask=blend_mask)
        blend_color = cv2.bitwise_and(hsv_line, hsv_line, mask=yellow_mask)

        img_roi = cv2.cvtColor(blend_color, cv2.COLOR_HSV2BGR)
        
        # traffic light detect
        # self.img_traffic_crop = img.copy()[self.roi_traffic_light_left_x1:self.roi_traffic_light_left_x2, self.roi_traffic_light_left_y1:self.roi_traffic_light_left_y2]
        self.img_traffic_crop = img.copy()[self.roi_traffic_light_right_x1:self.roi_traffic_light_right_x2, self.roi_traffic_light_right_y1:self.roi_traffic_light_right_y2]
        hsv_traffic = cv2.cvtColor(self.img_traffic_crop, cv2.COLOR_BGR2HSV)
        # R, G, O, B
        # red_mask = cv2.inRange(hsv_traffic, self.red_lower, self.red_upper)
        # red_detect = np.any(red_mask)
        # green_mask = cv2.inRange(hsv_traffic, self.green_lower, self.green_upper)
        # green_detect = np.any(green_mask)
        # orange_mask = cv2.inRange(hsv_traffic, self.orange_lower, self.orange_upper)
        # orange_detect = np.any(orange_mask)
        blue_mask = cv2.inRange(hsv_traffic, self.blue_lower, self.blue_upper)
        blue_detect = np.any(blue_mask)

        self.light_color = ''
        # if red_detect:
        #     self.light_color = 'Red'
        # elif green_detect:
        #     self.light_color = 'Green'
        # elif orange_detect:
        #     self.light_color = 'Orange'    
        # elif blue_detect:
        #     self.light_color = 'Blue'

        # if blue_detect:
        #     self.light_color = 'Blue'

        # print(f"light_color={self.light_color}")

        # cv2.imshow("ROI", self.img_traffic_crop)
    
        # cv2.imshow("bule", blue_mask)
        # cv2.imshow("BGR", rgb_img)

        # cv2.imshow("BGR", self.img_line_crop)
        # cv2.imshow("yellow line", yellow_mask)
        # cv2.imshow("yellow line", _mask)

        # print(self.light_color)

        return img_roi

    # edge detect
    def Img_canny(self, img):
        img_roi = self.Color_detect(img)
        img_gray = cv2.cvtColor(img_roi, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (27, 27), 0)
        img_edge = cv2.Canny(np.uint8(img_blur), 10, 30)
        # cv2.imshow("HSV", img_roi)
        # cv2.imshow("EDGE", img_edge)
        return img_edge        


    # stopline detect
    def Stopline_detect(self, img):
        stopline = img.copy()[self.roi_line_up: , :]
        hsv_line = cv2.cvtColor(stopline, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv_line, self.white_lower, self.white_upper)
        blend_color = cv2.bitwise_and(hsv_line, hsv_line, mask=white_mask)
        img_roi = cv2.cvtColor(blend_color, cv2.COLOR_HSV2BGR)
        # cv2.imshow("test", img_roi)
        
        return img_roi
    

    def Stopline_blur(self, img):
        stopline = self.Stopline_detect(img)
        stopline_gray = cv2.cvtColor(stopline, cv2.COLOR_BGR2GRAY)
        self.stopline_blur = cv2.GaussianBlur(stopline_gray, (27, 27), 0)
        # cv2.imshow("stopline blur", stopline)


    def Img_stopline(self):
        img_roi = self.stopline_blur.copy()[self.roi_stopline_x1:self.roi_stopline_x2, self.roi_stopline_y1:]
        ret, thresh = cv2.threshold(img_roi, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        stopline_count = cv2.countNonZero(thresh)
        # cv2.imshow("white", img_roi)
        # cv2.waitKey(1)
        if stopline_count > self.stopline_val:
            self.stopline_detct = True
        else:
            self.stopline_detct = False

    # Redzone
    def Img_redzone(self, img):
        self.img_line_crop = img.copy()[self.roi_line: , :]
        
        hsv_redzone = cv2.cvtColor(self.img_line_crop, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv_redzone, self.redzone_lower, self.redzone_upper)
        red_detect = np.any(red_mask)

        # cv2.imshow("red", red_mask)

        if red_detect:
            self.redzone_detect = True
        else:
            self.redzone_detect = False


    # edge cal
    def Line_detect(self, img):
        img_edge = self.Img_canny(img)
        edges = cv2.HoughLinesP(img_edge, 1, np.pi/360, 5, None, 10, 5)

        slopes = []
        lines = []
        seta = []

        if (type(edges) == type(None)):
            pass
        else:
            for edge in edges:
                x1, y1, x2, y2 = edge[0]

                if (abs(x2-x1) < 20):
                    slope = 0
                    deg = 0
                else:
                    slope = float(y2-y1) / float(x2-x1)
                    rad = atan(slope)
                    deg = rad * 180 / pi

                if (20 <= abs(deg)) :
                    lines.append(edge[0])
                    slopes.append(slope)
                    seta.append(deg)
            
            # left line list
            left_lines = []
            left_lines_x1 = []
            left_lines_x2 = []
            left_lines_y1 = []
            left_lines_y2 = []

            # right line list
            right_lines = []
            right_lines_x1 = []
            right_lines_x2 = []
            right_lines_y1 = []
            right_lines_y2 = []

            for idx, line in enumerate(lines):
                slope = slopes[idx]
                x1, y1, x2, y2 = line


                # left line
                if(slope < 0) and (x2 < 320) :
                    left_lines.append(line.tolist())

                    left_lines_x1.append(x1)
                    left_x1_avg = int(np.average(left_lines_x1))

                    left_lines_x2.append(x2)
                    left_x2_avg = int(np.average(left_lines_x2))

                    left_lines_y1.append(y1)
                    left_y1_avg = int(np.average(left_lines_y1))

                    left_lines_y2.append(y2)
                    left_y2_avg = int(np.average(left_lines_y2))

                    left_m = (left_y2_avg - left_y1_avg) / (left_x2_avg - left_x1_avg)

                    if -0.4 < left_m:
                    # if -0.4 < left_m:
                        slope = 0
                    
                    left_n = left_y1_avg - (left_m * left_x1_avg)

                    left_point_1 = int(-(left_n / left_m))
                    left_point_2 = int((180 - left_n) / left_m)
                    self.left_point_x = int((self.vertical_value - left_n) / left_m)
                    

                # right line
                if (slope > 0) and (x1 > 320):
                    right_lines.append(line.tolist())

                    right_lines_x1.append(x1)
                    right_x1_avg = int(np.average(right_lines_x1))

                    right_lines_x2.append(x2)
                    right_x2_avg = int(np.average(right_lines_x2))

                    right_lines_y1.append(y1)
                    right_y1_avg = int(np.average(right_lines_y1))

                    right_lines_y2.append(y2)
                    right_y2_avg = int(np.average(right_lines_y2))

                    right_m = (right_y2_avg - right_y1_avg) / (right_x2_avg - right_x1_avg)

                    right_n = right_y1_avg - (right_m * right_x1_avg)

                    right_point_1 = int(-(right_n / right_m))
                    right_point_2 = int((180 - right_n) / right_m)
                    self.right_point_x = int((self.vertical_value - right_n) / right_m)


            # print(f"leftM:{left_m}\t\t rightM:{right_m}")

            if self.left_point_x < 0:
                self.left_point_x = 0
            
            if self.right_point_x > 640:
                self.right_point_x = 640
                
            # vertical line
            cv2.line(self.img_line_crop, (0, self.vertical_value), (640, self.vertical_value), (0, 0, 255), 3)
            # left line draw
            if len(left_lines) == 0:
                cv2.line(self.img_line_crop, (0,0), (0,180), (255,0,255), 3)
            else:
                for line in left_lines:
                    left_x1_avg, left_x2_avg, left_y1_avg, left_y2_avg = line
                    cv2.line(self.img_line_crop, (left_point_1,0), (left_point_2,180), (125,0,0), 3)
                    cv2.circle(self.img_line_crop, (self.left_point_x,self.vertical_value), 10, (0,125,255), 3)
            
            # right line draw
            if len(right_lines) == 0:
                cv2.line(self.img_line_crop, (0,0), (0,180), (255,0,255), 3)
            else:
                for line in right_lines:
                    right_x1_avg, right_x2_avg, right_y1_avg, right_y2_avg = line
                    cv2.line(self.img_line_crop, (right_point_1,0), (right_point_2,180), (125,125,0), 3)
                    cv2.circle(self.img_line_crop, (self.right_point_x,self.vertical_value), 10, (0,255,0), 3)
                    
            # self.line_data.data = [self.left_point_x, self.right_point_x]
            
        # cv2.imshow("line", self.img_line_crop)
        # cv2.waitKey(1)


    def Cam_callback(self, data) :
        img = self.bridge.compressed_imgmsg_to_cv2(data)
        
        self.Line_detect(img)
        self.Stopline_blur(img)
        self.Img_stopline()
        self.Img_redzone(img)
        # Publish
        self.line_data.line_point = [self.left_point_x, self.right_point_x]
        self.line_data.light_color = self.light_color
        self.line_data.stopline_check = self.stopline_detct
        self.line_data.redzone_check = self.redzone_detect
        self.line_pub.publish(self.line_data)

if __name__ == "__main__":
    try:
        cr = CameraRec()
        rospy.spin()
    except rospy.ROSInitException:
        pass