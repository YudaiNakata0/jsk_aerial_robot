#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ROITracker():
    def __init__(self, topic, path, thres, flag):
        self.bridge = CvBridge()
        self.setup_parameters(topic, path, thres, flag)
        self.setup_ros()

    def setup_parameters(self, topic, path, thres, flag):
        self.image_topic = topic

        self.threshold = thres
        self.roi_image = None
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
        self.roi_size = 50
        self.click_point = None

        self.is_ROI_set = False
        self.read_roi_file_flag = flag
        if self.read_roi_file_flag:
            self.ref_image = cv2.imread(path, cv2.IMREAD_COLOR)
            if self.ref_image is False:
                rospy.logerr("cannot read template image")
            else:
                self.set_ROI(self.ref_image)
                print("read roi file")
                
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
            self.is_ROI_set = True

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not self.is_ROI_set:
            self.click_point = (x, y)

    def callback(self, msg):
        self.input_image(msg)
        print("get ros image")

        if self.frame is None:
            print("can't get frame")
            return

        cv2.imshow("tracking result", self.frame)
        cv2.setMouseCallback("tracking result", self.mouse_callback)
        cv2.waitKey(1)

        if not self.is_ROI_set and self.click_point is not None:
            print("wait for templete input")
            x, y = self.click_point
            h, w = self.frame.shape[:2]
            r = self.roi_size

            x1 = max(x - r, 0)
            y1 = max(y - r, 0)
            x2 = min(x + r, w)
            y2 = min(y + r, h)

            roi = self.frame[y1:y2, x1:x2].copy()

            if roi.size == 0:
                rospy.logwarn("Invalid ROI")
                return

            self.set_ROI(roi)
            self.is_ROI_set = True

            rospy.loginfo("ROI set at (%d, %d)", x, y)
            return

        if self.is_ROI_set and self.roi_image is not None:
            self.matching()
            self.show_roi()
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
        if self.roi_image is None:
            return
        self.result = cv2.matchTemplate(self.frame, self.roi_image, cv2.TM_CCOEFF_NORMED)
        self.min_val, self.max_val, self.min_loc, self.max_loc = cv2.minMaxLoc(self.result)
        self.score = self.max_val
        self.center[0] = self.max_loc[0] + 0.5*self.template_width
        self.center[1] = self.max_loc[1] + 0.5*self.template_height
        self.next_roi = self.frame[self.max_loc[1]:self.max_loc[1]+self.template_height, self.max_loc[0]:self.max_loc[0]+self.template_width].copy()
        self.roi_image = self.next_roi

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

    def show_roi(self):
        cv2.imshow("roi", self.roi_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("tracker_node")
    topic_name = rospy.get_param("/tracker_node/topic", "/usb_cam/image_raw")
    path = rospy.get_param("~path", "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/image/roi_ref.png")
    path = os.path.expanduser(path)
    thres = rospy.get_param("/tracker_node/thres", 0.8)
    read_roi_file_flag = rospy.get_param("/tracker_node/flag", True)
    try:
        tracker = ROITracker(topic=topic_name, path=path, thres=thres, flag=read_roi_file_flag)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
