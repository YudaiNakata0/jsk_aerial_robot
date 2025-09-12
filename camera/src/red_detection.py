#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import matplotlib.pyplot as plt

class ImageProcess():
    def __init__(self, input_img, file_name):
        self.img = input_img
        self.file_name = file_name
        self.lh1 = 0
        self.lh2 = 170
        self.ls = 100
        self.lv = 100
        self.uh1 = 10
        self.uh2 = 180
        self.us = 255
        self.uv = 255

    def convert_format(self):
        self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

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
        plt.imshow(cv2.cvtColor(masked_img, cv2.COLOR_BGR2RGB))
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
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=10)
        print(lines)
        if lines is not None:
            for x1, y1, x2, y2 in lines[0]:
                print(x1, y1, x2, y2)
                cv2.line(self.mask, (x1, y1), (x2, y2), (255, 0, 0), 4)
            plt.imshow(self.mask)
            plt.show()


    def draw_line(self):
        p1 = 100, 100
        p2 = 300, 300
        color = 255, 0, 0
        width = 3
        cv2.line(self.img, p1, p2, color, width)
        plt.imshow(self.img)
        plt.show()
        
if __name__ == "__main__":
    args = sys.argv
    file_name = args[1]
    input_img = cv2.imread(file_name)
    Image = ImageProcess(input_img=input_img, file_name=file_name)
    Image.convert_format()
    Image.generate_mask()
    #Image.mask_image()
    #Image.get_line()
    #Image.show_mask()
    Image.draw_line()
    Image.save_mask()
