#!/usr/bin/env python3
import cv2
import numpy as np
import matplotlib.pyplot as plt

class ImageProcess():
    def __init__(self, input_img):
        self.img = input_img
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
    

if __name__ == "__main__":
    input_img = cv2.imread("test.JPG")
    Image = ImageProcess(input_img=input_img)
    Image.convert_format()
    Image.generate_mask()
    Image.mask_image()
