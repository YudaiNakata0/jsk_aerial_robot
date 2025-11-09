#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ROITracker():
    def __init__(self, topic, path):
        self.bridge = CvBridge()
        self.setup_parameters(topic, path)
        self.setup_ros()

    def setup_parameters(self ,topic, path):
        self.image_topic = topic
        self.ref_image = cv2.imread(path, cv2.IMREAD_COLOR)
        if self.ref_image is None:
            rospy.logerr("cannot read template image")

        self.roi_image = []
        self.frame = []
        self.result = []
        self.min_val = 0.0
        self.max_val = 0.0
        self.min_loc = [0.0, 0.0]
        self.max_loc = [0.0, 0.0]
        self.center = [0.0, 0.0]

        self.score = 0.0
        self.template_width = 0.0
        self.template_height = 0.0
        self.top_left = [0.0, 0.0]
        self.bottom_right = [0.0, 0.0]
        
    def setup_ros(self):
        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.callback)
        self.pub_result = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)

    def callback(self, msg):
        self.input_image(msg)
        self.set_ROI()
        self.matching()
        print("score: " + str(self.score))
        if self.score > 0.8:
            self.draw_result()
            print("Found target")
            self.publish_center()
        else:
            print("No matched area")

    def input_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("conversio error: %s", str(e))
            return

    def set_ROI(self):
        self.roi_image = self.ref_image
        self.template_width, self.template_height = self.roi_image.shape[1], self.roi_image.shape[0]
        
    def matching(self):
        self.result = cv2.matchTemplate(self.frame, self.roi_image, cv2.TM_CCOEFF_NORMED)
        self.min_val, self.max_val, self.min_loc, self.max_loc = cv2.minMaxLoc(self.result)
        self.score = self.max_val
        self.center[0] = self.max_loc[0] + 0.5*self.template_width
        self.center[1] = self.max_loc[1] + 0.5*self.template_height

    def draw_result(self):
        self.top_left = self.max_loc
        self.bottom_right[0] = self.top_left[0] + self.template_width
        self.bottom_right[1] = self.top_left[1] + self.template_height
        cv2.rectangle(self.frame, self.top_left, self.bottom_right, (0, 255, 0), 3)
        cv2.imshow("tracking result", self.frame)
        cv2.waitKey(1)

    def publish_center(self):
        msg = Vector3()
        msg.x = self.center[0]
        msg.y = self.center[1]
        self.pub_result.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node("tracker_node")
    topic_name = rospy.get_param("~topic", "/usb_cam/image_raw")
    path = rospy.get_param("~path", "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/image/roi_ref.png")
    path = os.path.expanduser(path)
    try:
        ROITracker(topic=topic_name, path=path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
