#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage

class Image():
    def __init__(self):
        self.sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback )

    def get_image(self, msg):
        array = np.frombuffer(msg.data, dtype=np.uint8)
        self.image = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        
    def show_image(self):
        cv2.imshow("Neutral Image", self.image)
        cv2.waitKey(1)
        
    def callback(self, msg):
        self.get_image(msg)
        self.show_image()

if __name__ == "__main__":
    rospy.init_node("check_image_node")
    Class = Image()
    rospy.spin()
