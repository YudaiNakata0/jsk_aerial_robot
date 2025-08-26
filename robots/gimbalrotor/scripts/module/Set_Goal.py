#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from module import convert_to_quaternion as cq

class SetGoal():
    def __init__(self):
        self.d = 0.337
        self.h = 0.195
        self.epsilon = 0.1
        self.state = 0
        self.end_effector_pose = Pose()
        self.cog_pose = Pose()
        self.target = Vector3()
        self.direction = Vector3()
        self.direction_yaw = 0.0
        self.sub_current_pose = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_get_cog_current_pose)
        self.sub_goal_pose = rospy.Subscriber("/set_goal", Vector3, self.cb_set_goal_pose)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher("/end_effector/pose", Pose, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_update_end_effector_pose)

    def cb_update_end_effector_pose(self, event):
        self.get_end_effector_current_pose()

    def get_end_effector_current_pose(self):
        try:
            trans = self.tfBuffer.lookup_transform("world", "gimbalrotor/end_effector", rospy.Time(0))
            self.end_effector_pose.position = trans.transform.translation
            self.end_effector_pose.orientation = trans.transform.rotation
            #rospy.loginfo("End effector position: %s", self.end_effector_pose.position)
            #rospy.loginfo("End effector orientation: %s", self.end_effector_pose.orientation)
            self.state_pub.publish(self.end_effector_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error\n")
        
    def cb_get_cog_current_pose(self, msg):
        self.cog_pose = msg.pose.pose
        
    def cb_set_goal_pose(self, msg):
        self.target.x = msg.x
        self.target.y = msg.y
        pub_msg = self.calc_cog_goal_pose(msg)
        self.publish(pub_msg)
        self.state += 1

    def publish(self, msg):
        self.pub.publish(msg)

    def calc_cog_goal_pose(self, msg):
        #self.get_end_effector_current_pose()
        self.direction.x = msg.x - self.end_effector_pose.position.x
        self.direction.y = msg.y - self.end_effector_pose.position.y
        
        self.direction_yaw = math.atan2(self.direction.y, self.direction.x)
        yaw_degree = self.direction_yaw * 180 / math.pi
        rospy.loginfo("yaw angle: %s", yaw_degree)
        quaternion = cq.euler_to_quaternion(np.array([0, 0, yaw_degree]))
        
        pub_msg = PoseStamped()
        pub_msg.pose.position.x = msg.x - (self.d * math.cos(self.direction_yaw))
        pub_msg.pose.position.y = msg.y - (self.d * math.sin(self.direction_yaw))
        pub_msg.pose.position.z = 1.2
        pub_msg.pose.orientation = quaternion
        return pub_msg

    def detach(self):
        pub_msg = PoseStamped()
        #x = self.target.x - (self.epsilon * self.direction.x)
        #y = self.target.y - (self.epsilon * self.direction.y)
        pub_msg.pose.position.x = self.target.x - ((self.d + self.epsilon) * math.cos(self.direction_yaw))
        pub_msg.pose.position.y = self.target.y - ((self.d + self.epsilon) * math.sin(self.direction_yaw))
        pub_msg.pose.position.z = 1.2
        rospy.loginfo(pub_msg.pose.position)
        pub_msg.pose.orientation = self.cog_pose.orientation
        self.publish(pub_msg)
        self.state += 1
