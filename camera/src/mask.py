#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import os
import numpy as np
from sensor_msgs.msg import Image, CompressedImage

class GenerateMask():
    def __init__(self,lh,uh,ll,ul,ls,us):
        self.time = rospy.get_time()
        self.bridge = CvBridge()
        self.setup_hls_param(lh, uh, ll, ul, ls, us)
        self.image = Image()
        self.image_hls = Image()

        self.setup_ros()

    def setup_hls_param(self, lh, uh, ll, ul, ls, us):
        self.lh = lh
        self.uh = uh
        self.ll = ll
        self.ul = ul
        self.ls = ls
        self.us = us

    def setup_ros(self):
        self.sub_raw = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)
        self.pub_mask = rospy.Publisher("/processed_image/mask", Image, queue_size=1)
        self.pub_mask_cleaned = rospy.Publisher("/processed_image/mask_cleaned", Image, queue_size=1)
        self.pub_filtered_mask = rospy.Publisher("/processed_image/filtered_mask", Image, queue_size=1)

    def callback(self, msg):
        if rospy.get_time() - self.time < 0.1:
            return
        self.time = rospy.get_time()
        self.get_image(msg)
        self.generate_mask()
        self.clean_mask()
        self.cut_image()
        self.publish_mask()
        
    def get_image(self, image):
        array = np.frombuffer(image.data, dtype=np.uint8)
        self.image = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        self.image_hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)        

    def generate_mask(self):
        lowerb = (self.lh, self.ll, self.ls)
        upperb = (self.uh, self.ul, self.us)
        mask = cv2.inRange(self.image_hls, lowerb, upperb)
        self.mask = mask

    def clean_mask(self):
        kernel = np.ones((3, 3), np.uint8)
        self.mask_cleaned = cv2.morphologyEx(self.mask, cv2.MORPH_OPEN, kernel)

    def cut_image(self):
        path = os.path.expanduser("~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/masked_calib_white.png")
        cut_area = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        kernel = np.ones((5, 5), np.uint8)
        cut_area_dilated = cv2.dilate(cut_area, kernel, iterations=3)
        valid_area_mask = cv2.bitwise_not(cut_area_dilated)
        # print("mask: ", self.mask.shape)
        # print("valid_area_mask: ", valid_area_mask.shape)
        self.filtered_mask = cv2.bitwise_and(self.mask, valid_area_mask)

    def publish_mask(self):
        mask_msg = self.bridge.cv2_to_imgmsg(self.mask, encoding="mono8")
        mask_cleaned_msg = self.bridge.cv2_to_imgmsg(self.mask_cleaned, encoding="mono8")
        filtered_mask_msg = self.bridge.cv2_to_imgmsg(self.filtered_mask, encoding="mono8")
        self.pub_mask.publish(mask_msg)
        self.pub_mask_cleaned.publish(mask_cleaned_msg)
        self.pub_filtered_mask.publish(filtered_mask_msg)

        
if __name__ == "__main__":
    rospy.init_node("generate_mask_node")
    lh = rospy.get_param("/generate_mask_node/lowerb_hue", 0)
    uh = rospy.get_param("/generate_mask_node/upperb_hue", 10)
    ll = rospy.get_param("/generate_mask_node/lowerb_luminance", 50)
    ul = rospy.get_param("/generate_mask_node/upperb_luminance", 255)
    ls = rospy.get_param("/generate_mask_node/lowerb_saturation", 50)
    us = rospy.get_param("/generate_mask_node/upperb_saturation", 255)

    Mask = GenerateMask(lh, uh, ll, ul, ls, us)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

        
