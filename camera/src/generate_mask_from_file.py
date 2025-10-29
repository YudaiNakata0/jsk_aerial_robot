#!/usr/bin/env python3
import cv2
import sys
from cv_bridge import CvBridge
from mask import *

if __name__=="__main__":
    file_name = input("File name: ")
    parameters = input("Mask parameters: ").split()
    if parameters == None:
        generator = MaskGenerator(0, 179, 0, 50, 0, 255, single_flag=1)
    else:
        lh = int(parameters[0])
        uh = int(parameters[1])
        ll = int(parameters[2])
        ul = int(parameters[3])
        ls = int(parameters[4])
        us = int(parameters[5])
        generator = MaskGenerator(lh, uh, ll, ul, ls, us, single_flag=1)
    generator.generate_single_mask(file_name)
