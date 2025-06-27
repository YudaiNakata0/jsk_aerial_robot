#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from aerial_robot_msgs.msg import FlightNav

class MyClass():
    def __init__(self):
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.z_goal = 0.0
        self.subscriber = rospy.Subscriber("/mocap/pose", PoseStamped, self.cb_move_to_goal)
        self.publisher = rospy.Publisher("/beetle2/uav/nav", FlightNav, queue_size=10)

    def cb_move_to_goal(self, msg):
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y
        self.z_goal = msg.pose.position.z
        goal_msg = FlightNav()
        goal_msg.pos_xy_nav_mode = 2
        goal_msg.pos_z_nav_mode = 2
        goal_msg.target_pos_x = self.x_goal
        goal_msg.target_pos_y = self.y_goal - 1.0
        goal_msg.target_pos_z = self.z_goal
        self.publisher.publish(goal_msg)

if __name__=="__main__":
    rospy.init_node("test_node")
    myclass = MyClass()
    while not rospy.is_shutdown():
        rospy.spin()
