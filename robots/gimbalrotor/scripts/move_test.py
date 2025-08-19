#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from module import convert_to_quaternion as cq

class MyClass():
    def __init__(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.sub_current_pose = rospy.Subscriber("/gimbalrotor/mocap/pose", PoseStamped, self.cb_get_current_pose)
        self.sub_goal_pose = rospy.Subscriber("/set_goal", Vector3, self.cb_set_goal_pose)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    def cb_get_current_pose(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
    def cb_set_goal_pose(self, msg):
        pub_msg = self.calc_goal_pose(msg)
        self.publish(pub_msg)

    def publish(self, msg):
        self.pub.publish(msg)

    def calc_goal_pose(self, msg):
        direction_x = msg.x - self.current_x
        direction_y = msg.y - self.current_y
        yaw = math.atan2(direction_y, direction_x) * 180 / math.pi
        rospy.loginfo("yaw angle: %s", yaw)
        quaternion = cq.euler_to_quaternion(np.array([0, 0, yaw]))
        pub_msg = PoseStamped()
        pub_msg.pose.position.x = msg.x
        pub_msg.pose.position.y = msg.y
        pub_msg.pose.position.z = 1.2
        pub_msg.pose.orientation.x = quaternion.x
        pub_msg.pose.orientation.y = quaternion.y
        pub_msg.pose.orientation.z = quaternion.z
        pub_msg.pose.orientation.w = quaternion.w
        return pub_msg

if __name__ == "__main__":
    rospy.init_node("test_node")
    myclass = MyClass()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
