#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage
from pylsd.lsd import lsd

class ImageProcess():
    def __init__(self,lh1=0,lh2=170,uh1=10,uh2=180,ll=50,ul=255,ls=50,us=255,th=20,minll=40,maxlg=20):
        self.time = rospy.get_time()
        self.threshold_length = 400
        self.lh1 = lh1
        self.lh2 = lh2
        self.uh1 = uh1
        self.uh2 = uh2
        self.ll = ll
        self.ul = ul
        self.ls = ls
        self.us = us
        self.th = th
        self.minll = minll
        self.maxlg = maxlg
        self.nozzle_x = 695
        self.nozzle_y = 300
        self.window_x = 200
        self.window_y = 100
        self.error_x = 40
        self.sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)
        self.pub_cut = rospy.Publisher("/processed_image/cut", Image, queue_size=1)
        self.pub_mask = rospy.Publisher("/processed_image/mask", Image, queue_size=1)
        self.pub_lines = rospy.Publisher("/processed_image/lines", Image, queue_size=1)
        self.bridge = CvBridge()

    def get_image(self, msg):
        array = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        self.img = img[0 : self.nozzle_y+self.window_y, self.nozzle_x-self.window_x : self.nozzle_x+self.window_x]
        topic_image = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
        self.pub_cut.publish(topic_image)

    def show_image(self):
        cv2.imshow("Neutral Image", self.img)
        #cv2.resizeWindow("Neutral Image", 650, 450)
        cv2.moveWindow("Neutral Image", 100, 100)
        cv2.waitKey(1)
        
    def convert_format(self):
        self.img_hls = cv2.cvtColor(self.img, cv2.COLOR_BGR2HLS)

    def generate_mask(self):
        lowerb_1 = (self.lh1, self.ll, self.ls)
        lowerb_2 = (self.lh2, self.ll, self.ls)
        upperb_1 = (self.uh1, self.ul, self.us)
        upperb_2 = (self.uh2, self.ul, self.us)
        mask_1 = cv2.inRange(self.img_hls, lowerb_1, upperb_1)
        mask_2 = cv2.inRange(self.img_hls, lowerb_2, upperb_2)
        self.mask = mask_1 | mask_2

    def mask_image(self):
        masked_img = cv2.bitwise_and(self.img, self.img, mask=self.mask)
        plt.imshow(masked_img)
        plt.show()

    def show_mask(self):
        #cv2.imshow("Masked Image", self.mask)
        #cv2.resizeWindow("Masked Image", 650, 450)
        #cv2.moveWindow("Masked Image", 510, 100)
        #cv2.waitKey(1)
        topic_image = self.bridge.cv2_to_imgmsg(self.mask, encoding="mono8")
        self.pub_mask.publish(topic_image)

    def save_mask(self):
        new_name = "masked_" + self.file_name
        cv2.imwrite(new_name, self.mask)
    
    def select_lines(self, line):
        if line[1] < self.nozzle_y and line[3] < self.nozzle_y:
            if self.window_x-self.error_x < line[0] and line[0] < self.window_x+self.error_x and self.window_x-self.error_x < line[2] and line[2] < self.window_x+self.error_x:
                return 1
        return 0
        
    def get_line(self):
        edges = cv2.Canny(self.mask, 0, 255)
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/360, threshold=self.th, minLineLength=self.minll, maxLineGap=self.maxlg)
        print(lines)
        line_image = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2RGB)
        #cv2.rectangle(line_image, (self.window_x-self.error_x, 0), (self.window_x+self.error_x, self.nozzle_y), (0, 255, 0), 3)
        cv2.circle(line_image, (self.window_x, self.nozzle_y), 5, (0, 255, 0), -1)
        if lines is not None:
            for line in lines[0]:
                if self.select_lines(line):
                    x1, y1, x2, y2 = map(int, line[:4])
                    print(x1, y1, x2, y2)
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
        cv2.imshow("Lines_hough", line_image)
        #cv2.resizeWindow("Lines_hough", 650, 450)
        cv2.moveWindow("Lines_hough", 510, 100)
        cv2.waitKey(1)
        topic_image = self.bridge.cv2_to_imgmsg(line_image, encoding="bgr8")
        self.pub_lines.publish(topic_image)

    def select_lines_lsd(self, line):
        length = (line[0] - line[2])**2 + (line[1] - line[3])**2
        if length < self.threshold_length:
            return 0
        else:
            return 1
        
    def get_line_lsd(self):
        edges = cv2.Canny(self.mask, 0, 255)
        lines = lsd(self.mask)
        print(lines)
        line_image = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2RGB)
        for line in lines:
            if self.select_lines_lsd(line):
                x1, y1, x2, y2 = map(int, line[:4])
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
        cv2.imshow("Lines_lsd", line_image)
        #cv2.moveWindow("Lines_lsd", 750, 100)
        cv2.waitKey(1)

    def draw_line(self):
        p1 = 100, 100
        p2 = 300, 300
        color = 255, 0, 0
        width = 3
        cv2.line(self.img, p1, p2, color, width)
        plt.imshow(self.img)
        plt.show()

    def callback(self, msg):
        if rospy.get_time() - self.time < 0.1:
            return
        self.time = rospy.get_time()
        self.get_image(msg)
        self.show_image()
        self.convert_format()
        self.generate_mask()
        self.show_mask()
        self.get_line()
        #self.get_line_lsd()
        
if __name__ == "__main__":
    rospy.init_node("camera_processing_node")
    lh1 = rospy.get_param("/camera_processing_node/lowerb_hue_1", 0)
    lh2 = rospy.get_param("/camera_processing_node/lowerb_hue_2", 170)
    uh1 = rospy.get_param("/camera_processing_node/upperb_hue_1", 10)
    uh2 = rospy.get_param("/camera_processing_node/upperb_hue_2", 180)
    ll = rospy.get_param("/camera_processing_node/lowerb_luminance", 50)
    ul = rospy.get_param("/camera_processing_node/upperb_luminance", 255)
    ls = rospy.get_param("/camera_processing_node/lowerb_saturation", 50)
    us = rospy.get_param("/camera_processing_node/upperb_saturation", 255)
    th = rospy.get_param("/camera_processing_node/threshold", 20)
    minll = rospy.get_param("/camera_processing_node/minLineLength", 40)
    maxlg = rospy.get_param("/camera_processing_node/maxLineGap", 20)
    Class = ImageProcess(lh1,lh2,uh1,uh2,ll,ul,ls,us,th,minll,maxlg)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

