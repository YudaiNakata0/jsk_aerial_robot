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
from beetle.assembly_api import AssembleDemo
from beetle.disassembly_api import DisassembleDemo
from task.assembly_motion import *


class PolynomialTrajectory:
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  

    def compute_coefficients(self, start, target):
        T = self.duration
        A = np.array([
            [0,         0,      0,    0,  0, 1],
            [T**5,     T**4,   T**3,  T**2, T, 1],
            [0,         0,      0,    0,  1, 0],
            [5*T**4,   4*T**3, 3*T**2, 2*T, 1, 0],
            [0,         0,      0,    2,   0, 0],
            [20*T**3, 12*T**2, 6*T,    2,   0, 0]
        ])
        B = np.array([start, target, 0, 0, 0, 0])
        return np.linalg.solve(A, B)

    def generate_trajectory(self, start_pos, target_pos):
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        if elapsed_time > self.duration:
            return None 
        T = np.array([elapsed_time**5, elapsed_time**4, elapsed_time**3, 
                      elapsed_time**2, elapsed_time, 1])
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T)
        else:
            return (
                np.dot(self.coeffs_x, T),
                np.dot(self.coeffs_y, T),
                np.dot(self.coeffs_z, T)
            )

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

class AssembleState(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Module IDs
        modules_str = rospy.get_param("~module_ids", "")
        real_machine = rospy.get_param("~real_machine", True)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")

        # AssembleDemo 
        self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
    
    def execute(self, userdata):
        rospy.loginfo("Assembling UAVs...")
        try:
            self.assemble_demo.main()
            rospy.loginfo("Assembly completed successfully.")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process failed.")
            return 'failed'


def main():
    rospy.init_node('valve_task_state_machine')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    with sm:
        smach.StateMachine.add('ASSEMBLE', AssembleState(),
                                 transitions={'succeeded': 'TASK_COMPLETED',
                                              'failed': 'TASK_FAILED'})
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

