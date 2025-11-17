#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ROITracker():
    def __init__(self, topic, path, thres):
        self.bridge = CvBridge()
        self.setup_parameters(topic, path, thres)
        self.setup_ros()

    def setup_parameters(self ,topic, path, thres):
        self.image_topic = topic

        self.threshold = thres
        self.roi_image = []
        self.frame = None
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

        self.ref_image = cv2.imread(path, cv2.IMREAD_COLOR)
        if self.ref_image is False:
            rospy.logerr("cannot read template image")
        else:
            self.set_ROI(self.ref_image)
        self.is_ROI_set = False
        rospy.loginfo("Started terget tracking. Waiting for selecting ROI...")
        
    def setup_ros(self):
        self.sub_image = rospy.Subscriber(self.image_topic, Image, self.callback)
        self.pub_result = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)

    def generate_ROI_loop(self):
        if self.frame is not None and not self.is_ROI_set:
            cv2.imshow("Select ROI", self.frame)
            cv2.waitKey(1)
            
            roi = cv2.selectROI("Select ROI", self.frame, False)
            cv2.destroyWindow("Select ROI")
            
            x, y, w, h = map(int, roi)
            self.set_ROI(self.frame[y:y+h, x:x+w])
            tracker.is_ROI_set = True
        
    def callback(self, msg):
        self.input_image(msg)

        if not self.is_ROI_set:
            cv2.imshow("tracking result", self.frame)
            cv2.waitKey(1)
            return
        # if not self.is_ROI_set:
        #     roi = cv2.selectROI("Select ROI", self.frame, fromCenter=False)
        #     x, y, w, h = map(int, roi)
        #     if w == 0 or h == 0:
        #         print("select ROI")
        #         cv2.imshow("tracking result", self.frame)
        #         cv2.waitKey(1)
        #         return
        #     else:
        #         roi_image = self.frame[y:y+h, x:x+w].copy()
        #         self.set_ROI(roi_image)
        #         self.is_ROI_set = True
        #         cv2.imshow("ROI", self.next_roi)
        #         cv2.waitKey(1)
                
        self.matching()
        print("score: " + str(self.score))
        if self.score > self.threshold:
            self.draw_result()
            print("Found target")
            self.publish_center()
            # self.set_ROI(self.next_roi)
            # cv2.imshow("ROI", self.next_roi)
            # cv2.waitKey(1)
        else:
            print("No matched area")
            cv2.imshow("tracking result", self.frame)
            cv2.waitKey(1)

    def input_image(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("conversio error: %s", str(e))
            return

    def set_ROI(self, ref_image):
        if ref_image is None:
            return
        else:
            self.roi_image = ref_image
            self.template_width, self.template_height = self.roi_image.shape[1], self.roi_image.shape[0]
        
    def matching(self):
        self.result = cv2.matchTemplate(self.frame, self.roi_image, cv2.TM_CCOEFF_NORMED)
        self.min_val, self.max_val, self.min_loc, self.max_loc = cv2.minMaxLoc(self.result)
        self.score = self.max_val
        self.center[0] = self.max_loc[0] + 0.5*self.template_width
        self.center[1] = self.max_loc[1] + 0.5*self.template_height
        self.next_roi = self.frame[self.max_loc[1]:self.max_loc[1]+self.template_height, self.max_loc[0]:self.max_loc[0]+self.template_width].copy()

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
    thres = rospy.get_param("~thres", 0.8)
    try:
        tracker = ROITracker(topic=topic_name, path=path, thres=thres)
        while not rospy.is_shutdown():
            tracker.generate_ROI_loop()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
