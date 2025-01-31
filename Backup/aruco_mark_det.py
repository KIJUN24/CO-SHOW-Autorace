#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2.aruco as aruco

class CameraRec :
    def __init__(self):
        rospy.init_node("Camera_rec")
        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.Cam_callback)
        self.bridge = CvBridge()


    def Aruco_detect(self, img):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        parameters = aruco.DetectorParameters_create()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i in range(len(ids)):
                print(f"Detected marker ID: {ids[i]} at {corners[i]}")
                aruco.drawDetectedMarkers(img, corners, ids)

        cv2.imshow("Frame", img)

    def Cam_callback(self, data) :
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        self.Aruco_detect(img)


if __name__ == "__main__":
    try:
        cr = CameraRec()
        rospy.spin()
    except rospy.ROSInitException:
        pass