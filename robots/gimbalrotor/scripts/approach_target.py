#!/usr/bin/env python3
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3

class ImageBaseApproach():
    def __init__(self, x, y, c):
        self.setup_parameters(x, y, c)
        self.setup_ros()

    def setup_parameters(self, x, y, c):
        self.endeffector_xi = x
        self.endeffector_yi = y
        self.target_xi = 0.0
        self.target_yi = 0.0
        self.nav_coefficient = c
        
    def setup_ros(self):
        self.sub_target = rospy.Subscriber("/target/circle", Vector3, self.callback)
        self.pub_nav = rospy.Publisher("/gimbalrotor/uav/nav", FlightNav, queue_size=1)

    def callback(self, msg):
        self.target_xi = msg.x
        self.target_yi = msg.y
        error_xi = self.target_xi - self.endeffector_xi
        error_yi = self.target_yi - self.endeffector_yi
        nav_y = -error_xi * self.nav_coefficient
        nav_z = -error_yi * self.nav_coefficient
        self.publish(nav_y, nav_z)

    def publish(self, nav_y, nav_z):
        pub_msg = FlightNav()
        pub_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        pub_msg.pos_z_nav_mode = FlightNav.VEL_MODE
        pub_msg.target_vel_y = nav_y
        pub_msg.target_vel_z = nav_z
        rospy.loginfo("publish message to uav/nav: %s, %s", nav_y, nav_z)
        self.pub_nav.publish(pub_msg)


if __name__ == "__main__":
    rospy.init_node("navigation_node")
    navigator = ImageBaseApproach(x=650, y=410, c=0.0001)
    rospy.spin()
