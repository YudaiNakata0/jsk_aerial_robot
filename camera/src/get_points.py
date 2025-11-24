#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

class GetTarget():
    def __init__(self):
        self.bridge = CvBridge()
        self.points = []
        self.number_of_points = 0
        self.image_raw = Image()
        self.setup_ros()

    def setup_ros(self):
        self.sub_mask = rospy.Subscriber("/processed_image/filtered_mask", Image, self.callback)
        self.pub_center_image = rospy.Publisher("/processed_image/center", Image, queue_size=1)
        self.pub_center_coords = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)

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
        self.number_of_points = len(self.points)
        print(self.number_of_points)

    def calculate_center(self):
        pub_coords_msg = Vector3()
        if self.number_of_points < 1000:
            print("No sufficient points.")
            pub_coords_msg.z = 1
            return
        else:
            center = np.mean(self.points, axis=0)
            print(center)
            center_image = cv2.cvtColor(self.mono, cv2.COLOR_GRAY2RGB)
            cv2.circle(center_image, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)
            pub_msg = self.bridge.cv2_to_imgmsg(center_image, encoding="bgr8")
            self.pub_center_image.publish(pub_msg)
            
            pub_coords_msg.x = center[0]
            pub_coords_msg.y = center[1]
        self.pub_center_coords.publish(pub_coords_msg)
    
if __name__ == "__main__":
    rospy.init_node("detect_target_node")
    Class = GetTarget()
    rospy.spin()
