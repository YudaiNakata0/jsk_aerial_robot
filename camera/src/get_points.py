#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image

class GetTarget():
    def __init__(self):
        self.bridge = CvBridge()
        self.points = []
        self.image_raw = Image()
        self.setup_ros()

    def setup_ros(self):
        self.sub_mask = rospy.Subscriber("/processed_image/filtered_mask", Image, self.callback)
        self.pub_center = rospy.Publisher("/processed_image/center", Image, queue_size=1)

    def callback(self, msg):
        self.get_image(msg)
        self.get_points()
        self.calculate_center()

    def get_image(self, image):
        self.image = image

    def get_points(self):
        self.mono = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="mono8")
        ys, xs = np.where(self.mono == 255)
        self.points = list(zip(xs, ys))

    def calculate_center(self):
        center = np.mean(self.points, axis=0)
        print(center)
        center_image = cv2.cvtColor(self.mono, cv2.COLOR_GRAY2RGB)
        cv2.circle(center_image, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
        pub_msg = self.bridge.cv2_to_imgmsg(center_image, encoding="bgr8")
        self.pub_center.publish(pub_msg)
    
if __name__ == "__main__":
    rospy.init_node("detect_target_node")
    Class = GetTarget()
    rospy.spin()
