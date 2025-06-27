#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3

class TestClass():
    def __init__(self):
        self.x_cur = 0.0
        self.y_cur = 0.0
        self.z_cur = 0.0
        self.sub_1 = rospy.Subscriber("/beetle1/uav/cog/odom", Odometry, self.callback_get_current_state)
        self.sub_2 = rospy.Subscriber("/mytopic", Vector3, self.callback_publishe)
        self.pub = rospy.Publisher("/beetle1/uav/nav", FlightNav, queue_size=10)

    def callback_get_current_state(self, msg):
        self.x_cur = msg.pose.pose.position.x
        self.y_cur = msg.pose.pose.position.y
        self.z_cur = msg.pose.pose.position.z

    def callback_publishe(self, msg):
        x_goal = self.x_cur + msg.x
        y_goal = self.y_cur + msg.y
        z_goal = self.z_cur + msg.z
        rospy.loginfo([x_goal, y_goal, z_goal])
        goal_msg = FlightNav()
        goal_msg.pos_xy_nav_mode = 2
        goal_msg.pos_z_nav_mode = 2
        goal_msg.target_pos_x = x_goal
        goal_msg.target_pos_y = y_goal
        goal_msg.target_pos_z = z_goal
        self.pub.publish(goal_msg)


if __name__=="__main__":
    rospy.init_node("test_node")
    myclass = TestClass()
    while not rospy.is_shutdown():
        rospy.spin()
