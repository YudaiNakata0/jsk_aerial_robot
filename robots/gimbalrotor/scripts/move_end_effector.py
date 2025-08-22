#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
from module import convert_to_quaternion as cq

class MyClass():
    def __init__(self):
        self.d = 0.337
        self.h = 0.195
        self.current_x_g = 0.0
        self.current_y_g = 0.0
        self.current_z_g = 0.0
        self.current_x_e = 0.0
        self.current_y_e = 0.0
        self.current_z_e = 0.0
        self.sub_current_pose = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_get_cog_current_pose)
        self.sub_goal_pose = rospy.Subscriber("/set_goal", Vector3, self.cb_set_goal_pose)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        #todo tfを読み取ってエンドエフェクタ位置を取得
    def get_end_effector_current_pose(self):
        
    def cb_get_cog_current_pose(self, msg):
        self.current_x_g = msg.pose.pose.position.x
        self.current_y_g = msg.pose.pose.position.y
        self.current_z_g = msg.pose.pose.position.z
        
    def cb_set_goal_pose(self, msg):
        pub_msg = self.calc_cog_goal_pose(msg)
        self.publish(pub_msg)

    def publish(self, msg):
        self.pub.publish(msg)

    def calc_cog_goal_pose(self, msg):
        direction_x = msg.x - self.current_x_g
        direction_y = msg.y - self.current_y_g
        
        yaw = math.atan2(direction_y, direction_x)
        yaw_degree = yaw * 180 / math.pi
        rospy.loginfo("yaw angle: %s", yaw_degree)
        quaternion = cq.euler_to_quaternion(np.array([0, 0, yaw_degree]))
        
        pub_msg = PoseStamped()
        pub_msg.pose.position.x = msg.x - (self.d * math.cos(yaw))
        pub_msg.pose.position.y = msg.y - (self.d * math.sin(yaw))
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
