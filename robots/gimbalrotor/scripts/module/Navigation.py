#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import numpy as np
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PoseStamped, Vector3, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from module import operation_quaternion as oq

class Navigation():
    def __init__(self, t=1.0, d=0.337, h=0.195, l=0.1):
        self.interval = t
        self.d = d
        self.h = h
        self.path_length = l
        self.part = 1
        self.state = 0
        """
        state
        0 : no direction
        1 : recieved direction(long way)
        2 : moving(long way)
        3 : completed move(long way)
        
        11: recieved direction(short way)
        12: moving(short way)
        13: completed move(short way)
        """

        self.msg_register = Vector3()
        self.endeffector_pose = Pose()
        self.endeffector_goal_pos = Vector3()
        self.cog_pose = Pose()
        self.cog_yaw = 0.0
        self.cog_start_pose =  Pose()
        self.cog_goal_pose = Pose()

        self.direction = Vector3()
        self.cog_direction = Vector3()
        self.direction_yaw = 0.0
        self.direction_yaw_degree = 0.0
        self.distance = 0.0
        self.yaw_difference = 0.0

        self.waypoint_list = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_update_endeffector_pose)
        self.sub_cog_current_pose = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_get_cog_current_pose)
        self.sub_endeffector_goal_pose = rospy.Subscriber("/set_goal", Vector3, self.cb_set_cog_goal_pose)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher("/end_effector/pose", Pose, queue_size=10)
        self.timer_waypoint = rospy.Timer(rospy.Duration(0.1), self.cb_generate_cog_waypoint_list)
        self.timer_publish = rospy.Timer(rospy.Duration(0.1), self.cb_publish_cog_waypoint)
        self.sub_contact = rospy.Subscriber("/read_sensor", String, self.cb_detect_contact)
        self.timer_state = rospy.Timer(rospy.Duration(0.1), self.cb_publish_state)
        self.pub_state = rospy.Publisher("/my_flight_state", Int8, queue_size=10)

    #エンドエフェクタの位置姿勢更新
    def cb_update_endeffector_pose(self, event):
        self.get_endeffector_current_pose()
    def get_endeffector_current_pose(self):
        try:
            trans = self.tfBuffer.lookup_transform("world", "gimbalrotor/end_effector", rospy.Time(0))
            self.endeffector_pose.position = trans.transform.translation
            self.endeffector_pose.orientation = trans.transform.rotation
            self.state_pub.publish(self.endeffector_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error\n")

    #コールバック:重心の位置姿勢更新(in:Pose)
    def cb_get_cog_current_pose(self, msg):
        self.cog_pose = msg.pose.pose
        angles = oq.quaternion_to_euler(self.cog_pose.orientation)
        self.cog_yaw = angles[2]

    #コールバック:重心の目標位置姿勢取得(in:Vector3),トピックの記録
    def cb_set_cog_goal_pose(self, msg):
        self.msg_register = msg
        self.set_cog_goal_pose(msg)
    #重心の目標位置姿勢取得(in:Vector3)
    def set_cog_goal_pose(self, msg):
        self.cog_start_pose = self.cog_pose
        self.set_direction(msg)
        if self.distance > self.path_length*2:
            self.calc_pose_endeffector_to_cog(msg)
            self.partition()
            self.state = 1
        else:
            self.simple_move()
    #目標へのベクトル,角度,距離計算(in:Vector3)
    def set_direction(self, vec):
        self.direction.x = vec.x - self.endeffector_pose.position.x
        self.direction.y = vec.y - self.endeffector_pose.position.y
        self.direction_yaw = np.arctan2(self.direction.y, self.direction.x)
        self.direction_yaw_degree = self.direction_yaw * 180 / np.pi
        self.yaw_difference = self.direction_yaw - self.cog_yaw
        self.distance = np.sqrt(self.direction.x ** 2 + self.direction.y ** 2)
        rospy.loginfo("distance: %s", self.distance)
    #エンドエフェクタの目標位置から重心の目標位置姿勢を計算(in:Vector3)
    def calc_pose_endeffector_to_cog(self, msg):
        quaternion = oq.euler_to_quaternion(np.array([0, 0, self.direction_yaw_degree]))
        self.cog_goal_pose.position.x = msg.x - (self.d * np.cos(self.direction_yaw))
        self.cog_goal_pose.position.y = msg.y - (self.d * np.sin(self.direction_yaw))
        self.cog_goal_pose.position.z = self.cog_start_pose.position.z
        self.cog_goal_pose.orientation = quaternion
        rospy.loginfo(self.cog_goal_pose)
        self.cog_direction.x = self.cog_goal_pose.position.x - self.cog_start_pose.position.x
        self.cog_direction.y = self.cog_goal_pose.position.y - self.cog_start_pose.position.y
        self.cog_direction.z = self.cog_goal_pose.position.z - self.cog_start_pose.position.z
    #目標への距離が小さい場合平行移動する重心の目標位置姿勢を計算
    def simple_move(self):
        if self.state != 0:
            return
        self.cog_goal_pose.position.x = self.cog_start_pose.position.x + self.direction.x
        self.cog_goal_pose.position.y = self.cog_start_pose.position.y + self.direction.y
        self.cog_goal_pose.position.z = self.cog_start_pose.position.z
        self.cog_goal_pose.orientation = self.cog_start_pose.orientation
        msg = PoseStamped()
        msg.pose = self.cog_goal_pose
        self.state = 12
        if self.state != 12:
            return
        self.pub.publish(msg)
        rospy.sleep(1.0)
        if self.state == 12:
            self.state = 0
            self.set_cog_goal_pose(self.msg_register)
        
    #軌道の分割数決定
    def partition(self):
        n = int(self.distance // self.path_length)
        if n > 1:
            self.part = n
        rospy.loginfo("partition number: %s", self.part)

    #rosタイマーコールバック:重心の経由点リストを計算
    def cb_generate_cog_waypoint_list(self, event):
        if self.state != 1:
            return
        self.generate_cog_waypoint_list()
        self.state += 1
    #重心の経由点リストを計算
    def generate_cog_waypoint_list(self):
        if self.state != 1:
            return
        self.waypoint_list.clear()
        for i in range(1, self.part+1):
            waypoint = self.calc_cog_waypoint_pose(i)
            self.waypoint_list.append(waypoint)
            rospy.loginfo("waypoint: %s", waypoint)
    #重心の経由点を計算(in:int)
    def calc_cog_waypoint_pose(self, i):
        if i > self.part:
            return
        i = float(i) / self.part
        cog_waypoint_pose = PoseStamped()
        cog_waypoint_pose.pose.position.x = self.cog_start_pose.position.x + (self.cog_direction.x * i)
        cog_waypoint_pose.pose.position.y = self.cog_start_pose.position.y + (self.cog_direction.y * i)
        cog_waypoint_pose.pose.position.z = self.cog_start_pose.position.z
        yaw_degree = (self.cog_yaw + self.yaw_difference * i) * 180 / np.pi
        cog_waypoint_pose.pose.orientation = oq.euler_to_quaternion(np.array([0, 0, yaw_degree]))
        return cog_waypoint_pose

    #rosタイマーコールバック:中間点の送信
    def cb_publish_cog_waypoint(self, event):
        if self.state != 2:
            return
        self.publish_cog_waypoint()
        if self.state == 2:
            self.state = 0
            self.set_cog_goal_pose(self.msg_register)
    #中間点の送信
    def publish_cog_waypoint(self):
        for i in range(0, self.part):
            if self.state != 2:
                return
            self.pub.publish(self.waypoint_list[i])
            rospy.sleep(self.interval)

    #コールバック:接触検知による状態の更新
    def cb_detect_contact(self, msg):
        if self.state == 2 or self.state == 12:
            self.state = 3

    #状態を送信
    def cb_publish_state(self, event):
        self.pub_state.publish(self.state)
        
