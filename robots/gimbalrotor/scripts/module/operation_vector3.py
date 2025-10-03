#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3

def get_difference(v1, v2):
    v_r = Vector3()
    v_r.x = v1.x - v2.x
    v_r.y = v1.y - v2.y
    v_r.z = v1.z - v2.z
    return v_r

def get_norm(v):
    d = np.sqrt(v.x**2 + v.y**2 + v.z**2)
    return d
