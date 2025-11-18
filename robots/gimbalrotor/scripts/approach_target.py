#!/usr/bin/env python3
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3, Pose, PoseStamped

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
        self.cog_pose = Pose()
        self.cog_goal_pose = Pose()
        self.endeffector_pose = Pose()
        self.endeffector_goal_pose = Pose()
        self.is_cog_goal_record = False
        
    def setup_ros(self):
        self.sub_target_circle = rospy.Subscriber("/target/2D_position", Vector3, self.callback)
        self.sub_endeffector = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Pose, self.cb_record_cog_pose)
        self.sub_endeffector = rospy.Subscriber("/gimbalrotor/endeffector_pose", Pose, self.cb_record_endeffector_pose)
        self.pub_nav = rospy.Publisher("/gimbalrotor/uav/nav", FlightNav, queue_size=1)
        self.pub_cog = rospy.Publisher("/gimbalrotor/target_pose", PoseStamped, queue_size=1)

    def callback(self, msg):
        self.target_xi = msg.x
        self.target_yi = msg.y
        error_xi = self.target_xi - self.endeffector_xi
        error_yi = self.target_yi - self.endeffector_yi
        error_di2 = error_xi**2 + error_yi**2
        if error_di2 < 10:
            print("approached successfully")
            self.cog_goal_pose = self.cog_pose
            self.endeffector_goal_pose = self.endeffector_pose
            self.is_cog_goal_record = True
        else:
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
        
    def cb_record_cog_pose(self, msg):
        self.cog_pose = msg
        
    def cb_record_endeffector_pose(self, msg):
        self.endeffector_pose = msg

if __name__ == "__main__":
    rospy.init_node("navigation_node")
    x = rospy.get_param("~ee_x", 650)
    y = rospy.get_param("~ee_y", 410)
    c = rospy.get_param("~coef", 0.0001)
    navigator = ImageBaseApproach(x=x, y=y, c=c)
    rospy.spin()
