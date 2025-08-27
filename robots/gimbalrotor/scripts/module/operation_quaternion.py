#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Quaternion

def euler_to_quaternion(euler_angles):
    rotation = Rotation.from_euler("xyz", euler_angles, degrees=True)
    quaternion = rotation.as_quat()
    msg = Quaternion()
    msg.x = quaternion[0]
    msg.y = quaternion[1]
    msg.z = quaternion[2]
    msg.w = quaternion[3]
    return msg

def quaternion_to_euler(quaternion):
    rotation = Rotation.from_quat(np.array([quaternion.x,
                                            quaternion.y,
                                            quaternion.z,
                                            quaternion.w]))
    euler_angles = rotation.as_euler("xyz")
    return euler_angles
