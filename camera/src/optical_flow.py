#!/usr/bin/env python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class OpticalFlowCalculator():
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_gray = None
        self.setup_ros()
        
    def setup_ros(self):
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return
        
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                            0.5, 3, 15, 3, 5, 1.2, 0)
        mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        hsv = np.zeros_like(frame)
        hsv[..., 1] = 255
        hsv[..., 0] = ang * 180 / np.pi / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.imshow('optical flow', bgr)
        cv2.waitKey(1)
        
        self.prev_gray = gray

if __name__ == "__main__":
    rospy.init_node('optical_flow_node')
    try:
        Flow = OpticalFlowCalculator()
        rospy.spin()
    except:
        pass
