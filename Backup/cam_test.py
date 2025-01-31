#!/usr/bin/env python3

import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class CameraRec :
    def __init__(self):
        rospy.init_node("Camera_rec", anonymous=True)
        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.callback)
        self.bridge = CvBridge()
        self.canny_pub = rospy.Publisher("Canny_img", CompressedImage, queue_size=5)

    def img_canny(self, img):
        img_crop = img
        cv2.imshow("test", img_crop)

    def callback(self, data) :
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        self.img_canny(img)
        cv2.imshow("test", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        cr = CameraRec()
        rospy.spin()
    except rospy.ROSInitException:
        pass