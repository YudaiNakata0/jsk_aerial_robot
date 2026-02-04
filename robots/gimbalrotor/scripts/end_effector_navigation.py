#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from module import operation_quaternion as oq

class EndEffectorNavigation():
    def __init__(self):
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.d = 0.337
        self.h = 0.118
        self.cog_pose = Pose()
        self.endeffector_pose = Pose()
        self.cog_goal_pose = Pose()
        self.direction = Vector3()
        self.direction_yaw = 0.0
        self.direction_yaw_degree = 0.0
        self.distance = 0.0
        self.mean_velocity = 0.1
        
    def setup_ros(self):
        self.sub_current_pose = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_get_cog_current_pose)
        self.sub_goal_pose = rospy.Subscriber("/set_goal_pose", Pose, self.cb_set_goal_pose)
        self.pub = rospy.Publisher("/gimbalrotor/target_pose", PoseStamped, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_update_endeffector_pose)
    

    def cb_update_endeffector_pose(self, event):
        self.get_endeffector_current_pose()

    def get_endeffector_current_pose(self):
        try:
            trans = self.tfBuffer.lookup_transform("world", "gimbalrotor/end_effector", rospy.Time(0))
            self.endeffector_pose.position = trans.transform.translation
            self.endeffector_pose.orientation = trans.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error\n")
        
    def cb_get_cog_current_pose(self, msg):
        self.cog_pose = msg.pose.pose
        
    def cb_set_goal_pose(self, msg):
        pub_msg = self.calc_cog_goal_pose(msg)
        self.publish(pub_msg)

    def publish(self, msg):
        self.pub.publish(msg)

    def calc_cog_goal_pose(self, msg):
        self.set_direction(msg)
        
        self.cog_goal_pose.position.x = msg.position.x - (self.d * np.cos(self.direction_yaw))
        self.cog_goal_pose.position.y = msg.position.y - (self.d * np.sin(self.direction_yaw))
        if msg.position.z > 1.0:
            self.cog_goal_pose.position.z = msg.position.z - self.h
        else:
            self.cog_goal_pose.position.z = self.cog_pose.position.z
        self.cog_goal_pose.orientation = msg.orientation

        duration = self.distance / self.mean_velocity
        rospy.loginfo("duration: %s", duration)

        pub_msg = PoseStamped()
        pub_msg.header.stamp.secs = int(rospy.get_time() + duration)
        pub_msg.pose = self.cog_goal_pose
        
        return pub_msg

    def set_direction(self, pose):
        self.direction.x = pose.position.x - self.endeffector_pose.position.x
        self.direction.y = pose.position.y - self.endeffector_pose.position.y
        self.direction.z = pose.position.z - self.endeffector_pose.position.z
        angles = oq.quaternion_to_euler(pose.orientation)
        self.direction_yaw = angles[2]
        self.direction_yaw_degree = self.direction_yaw * 180 / np.pi
        self.distance = np.sqrt(self.direction.x ** 2 + self.direction.y ** 2 + self.direction.z ** 2)
    
if __name__ == "__main__":
    rospy.init_node("navigation_node")
    myclass = EndEffectorNavigation()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
