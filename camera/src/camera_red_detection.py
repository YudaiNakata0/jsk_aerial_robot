#!/usr/bin/env python3
import cv2
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CompressedImage

class ImageProcess():
    def __init__(self):
        #self.img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        #self.file_name = file_name
        self.lh1 = 0
        self.lh2 = 170
        self.ls = 100
        self.lv = 100
        self.uh1 = 10
        self.uh2 = 180
        self.us = 255
        self.uv = 255
        self.sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)

    def get_image(self, msg):
        array = np.frombuffer(msg.data, dtype=np.uint8)
        self.img = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        
    def convert_format(self):
        self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_RGB2HSV)

    def generate_mask(self):
        lowerb_1 = (self.lh1, self.ls, self.lv)
        lowerb_2 = (self.lh2, self.ls, self.lv)
        upperb_1 = (self.uh1, self.us, self.uv)
        upperb_2 = (self.uh2, self.us, self.uv)
        mask_1 = cv2.inRange(self.img_hsv, lowerb_1, upperb_1)
        mask_2 = cv2.inRange(self.img_hsv, lowerb_2, upperb_2)
        self.mask = mask_1 | mask_2

    def mask_image(self):
        masked_img = cv2.bitwise_and(self.img, self.img, mask=self.mask)
        plt.imshow(masked_img)
        plt.show()

    def show_mask(self):
        plt.imshow(self.mask)
        plt.show()

    def save_mask(self):
        new_name = "masked_" + self.file_name
        cv2.imwrite(new_name, self.mask)

    def get_line(self):
        #lines = cv2.HoughLinesP(self.mask, rho=1, theta=np.pi/360, threshold=50, minLineLength=50, maxLineGap=5)
        #print(lines)
        edges = cv2.Canny(self.mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=30, maxLineGap=20)
        print(lines)
        line_image = cv2.cvtColor(self.mask, cv2.COLOR_GRAY2RGB)
        if lines is not None:
            for x1, y1, x2, y2 in lines[0]:
                print(x1, y1, x2, y2)
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
            cv2.imshow("window", line_image)
            #plt.imshow(line_image)
            #plt.show()
        #new_name = "linedetected_" + self.file_name
        #cv2.imwrite(new_name, line_image)


    def draw_line(self):
        p1 = 100, 100
        p2 = 300, 300
        color = 255, 0, 0
        width = 3
        cv2.line(self.img, p1, p2, color, width)
        plt.imshow(self.img)
        plt.show()

    def callback(self, msg):
        print("callback called\n")
        plt.clf()
        self.get_image(msg)
        self.convert_format()
        self.generate_mask()
        self.get_line()
        
if __name__ == "__main__":
    rospy.init_node("camera_processing_node")
    Class = ImageProcess()
    rospy.spin()

