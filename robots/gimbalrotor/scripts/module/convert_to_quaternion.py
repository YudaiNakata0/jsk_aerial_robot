#!/usr/bin/env python3
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Quaternion

def euler_to_quaternion(euler_angles):
    rot = Rotation.from_euler("xyz", euler_angles, degrees=True)
    qua = rot.as_quat()
    msg = Quaternion()
    msg.x = qua[0]
    msg.y = qua[1]
    msg.z = qua[2]
    msg.w = qua[3]
    return msg
