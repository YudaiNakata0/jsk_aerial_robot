#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time
import math
import threading
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from task.assembly_motion import *
from trajectory import PolynomialTrajectory

class MotionController:
    def __init__(self):
        pass
    def execute_poly_motion_pose_async(pub, start, target, avg_speed, rate_hz=20, delay_after=0):

        def motion():
            distance = math.sqrt((target[0]-start[0])**2 +
                                (target[1]-start[1])**2 +
                                (target[2]-start[2])**2)
            if avg_speed <= 0:
                speed = 0.1
            else:
                speed = avg_speed
            duration = distance / speed
            rospy.loginfo("Executing polynomial motion (PoseStamped) asynchronously: from [{:.3f}, {:.3f}, {:.3f}] to [{:.3f}, {:.3f}, {:.3f}] over {:.2f} s".format(
                start[0], start[1], start[2],
                target[0], target[1], target[2],
                duration))

            traj = PolynomialTrajectory(duration)
            traj.generate_trajectory(start, target)
            rate_obj = rospy.Rate(rate_hz)
            while not rospy.is_shutdown():
                pt = traj.evaluate()
                if pt is None:
                    break
                msg = PoseStamped()
                msg.pose.position.x = pt[0]
                msg.pose.position.y = pt[1]
                msg.pose.position.z = pt[2]
                pub.publish(msg)
                rate_obj.sleep()
            if delay_after > 0:
                time.sleep(delay_after)
        t = threading.Thread(target=motion)
        t.start()
        return t

    def execute_poly_motion_nav(pub, start, target, avg_speed, rate_hz=20, delay_after=0):
        distance = math.sqrt((target[0]-start[0])**2 +
                            (target[1]-start[1])**2 +
                            (target[2]-start[2])**2)
        if avg_speed <= 0:
            avg_speed = 0.1
        duration = distance / avg_speed
        rospy.loginfo("Executing polynomial motion (FlightNav): from {:.3f} to {:.3f} over {:.2f} s".format(start, target, duration))
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        rate_obj = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            pt = traj.evaluate()
            if pt is None:
                break
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = pt[0]
            msg.target_pos_y = pt[1]
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = pt[2]
            pub.publish(msg)
            rate_obj.sleep()
        if delay_after > 0:
            time.sleep(delay_after)



