#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage

class ColorJudge():
    def __init__(self,rlh1,rlh2,ruh1,ruh2,rll,rul,rls,rus,blh,buh,bll,bul,bls,bus):
        self.time = rospy.get_time()
        self.bridge = CvBridge()
        self.sub_raw = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)
        self.pub_cut = rospy.Publisher("/processed_image/cut", Image, queue_size=1)
        self.pub_red_mask = rospy.Publisher("/processed_image/red_mask", Image, queue_size=1)
        self.pub_blue_mask = rospy.Publisher("/processed_image/blue_mask", Image, queue_size=1)
        self.pub_blue_mask_cleaned = rospy.Publisher("/processed_image/blue_mask_cleaned", Image, queue_size=1)

        self.setup_hls_param_for_red(rlh1, rlh2, ruh1, ruh2, rll, rul, rls, rus)
        self.setup_hls_param_for_blue(blh, buh, bll, bul, bls, bus)

    def setup_hls_param_for_red(self, rlh1, rlh2, ruh1, ruh2, rll, rul, rls, rus):
        self.rlh1 = rlh1
        self.rlh2 = rlh2
        self.ruh1 = ruh1
        self.ruh2 = ruh2
        self.rll = rll
        self.rul = rul
        self.rls = rls
        self.rus = rus

    def setup_hls_param_for_blue(self, blh, buh, bll, bul, bls, bus):
        self.blh = blh
        self.buh = buh
        self.bll = bll
        self.bul = bul
        self.bls = bls
        self.bus = bus

    def get_image(self, image):
        array = np.frombuffer(image.data, dtype=np.uint8)
        self.image = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
	#self.image = image[0 : self.nozzle_y+self.window_y, self.nozzle_x-self.window_x : self.nozzle_x+self.window_x]
        topic_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        self.pub_cut.publish(topic_image)

    def show_image(self):
        cv2.imshow("Neutral Image", self.image)
        cv2.waitKey(1)

    def convert_format(self):
        self.image_hls = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)

    def generate_mask_red(self):
        lowerb_1 = (self.rlh1, self.rll, self.rls)
        lowerb_2 = (self.rlh2, self.rll, self.rls)
        upperb_1 = (self.ruh1, self.rul, self.rus)
        upperb_2 = (self.ruh2, self.rul, self.rus)
        mask_1 = cv2.inRange(self.image_hls, lowerb_1, upperb_1)
        mask_2 = cv2.inRange(self.image_hls, lowerb_2, upperb_2)
        self.red_mask = mask_1 | mask_2

    def generate_mask_blue(self):
        lowerb = (self.blh, self.bll, self.bls)
        upperb = (self.buh, self.bul, self.bus)
        mask = cv2.inRange(self.image_hls, lowerb, upperb)
        self.blue_mask = mask
        kernel = np.ones((3, 3), np.uint8)
        self.blue_mask_cleaned = cv2.morphologyEx(self.blue_mask, cv2.MORPH_OPEN, kernel)

    def show_mask(self):
        cv2.imshow("Red Masked Image", self.red_mask)
        cv2.imshow("Blue Masked Image", self.blue_mask)
        cv2.imshow("Blue Masked Image(cleaned)", self.blue_mask_cleaned)
        cv2.waitKey(1)
        topic_image_red = self.bridge.cv2_to_imgmsg(self.red_mask, encoding="mono8")
        topic_image_blue = self.bridge.cv2_to_imgmsg(self.blue_mask, encoding="mono8")
        topic_image_blue_cleaned = self.bridge.cv2_to_imgmsg(self.blue_mask_cleaned, encoding="mono8")
        self.pub_red_mask.publish(topic_image_red)
        self.pub_blue_mask.publish(topic_image_blue)
        self.pub_blue_mask_cleaned.publish(topic_image_blue_cleaned)
        
    def get_points(self):
        self.mono = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="mono8")
        ys, xs = np.where(self.mono == 255)
        self.points = list(zip(xs, ys))

    def callback(self, msg):
        if rospy.get_time() - self.time < 0.1:
            return
        self.time = rospy.get_time()
        self.get_image(msg)
        self.show_image()
        self.convert_format()
        self.generate_mask_red()
        self.generate_mask_blue()
        self.show_mask()
        #self.get_points()

if __name__ == "__main__":
    rospy.init_node("camera_processing_node")
    rlh1 = rospy.get_param("/camera_processing_node/red_lowerb_hue_1", 0)
    rlh2 = rospy.get_param("/camera_processing_node/red_lowerb_hue_2", 170)
    ruh1 = rospy.get_param("/camera_processing_node/red_upperb_hue_1", 10)
    ruh2 = rospy.get_param("/camera_processing_node/red_upperb_hue_2", 180)
    rll = rospy.get_param("/camera_processing_node/red_lowerb_luminance", 50)
    rul = rospy.get_param("/camera_processing_node/red_upperb_luminance", 255)
    rls = rospy.get_param("/camera_processing_node/red_lowerb_saturation", 50)
    rus = rospy.get_param("/camera_processing_node/red_upperb_saturation", 255)
    blh = rospy.get_param("/camera_processing_node/blue_lowerb_hue", 110)
    buh = rospy.get_param("/camera_processing_node/blue_upperb_hue", 130)
    bll = rospy.get_param("/camera_processing_node/blue_lowerb_luminance", 50)
    bul = rospy.get_param("/camera_processing_node/blue_upperb_luminance", 255)
    bls = rospy.get_param("/camera_processing_node/blue_lowerb_saturation", 50)
    bus = rospy.get_param("/camera_processing_node/blue_upperb_saturation", 255)

    Class = ColorJudge(rlh1,rlh2,ruh1,ruh2,rll,rul,rls,rus,blh,buh,bll,bul,bls,bus)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
