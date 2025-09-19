#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import numpy as np
from std_msgs.msg import String, Int8, Empty
from geometry_msgs.msg import PoseStamped, Vector3, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from module import operation_quaternion as oq

class Navigation():
    def __init__(self, t=1.0, d=0.337, h=0.195, l=0.1, m=0, e=0.02, v=0.05):
        self.interval = t
        self.d = d
        self.h = h
        self.path_length = l
        self.part = 1
        self.part_attach = 5
        self.epsilon = e
        self.mean_velocity = v
        self.state = 0
        """
        state
        0 : no direction
        1 : recieved direction(long way)
        2 : moving(long way)
        3 : completed moving(long way)
        4 : attaching
        5 : completed attaching

        11: recieved direction(short way)
        12: moving(short way)
        13: completed move(short way)
        """
        self.direction_mode = 0
        """
        direction_mode
        0 : position direction
        1 : pose direction
        """
        self.execution_mode = m
        """
        0 : real machine
        1 : simulation
        """

        self.msg_register = Vector3()
        self.msg_register_pose = Pose()
        self.endeffector_pose = Pose()
        self.endeffector_goal_pos = Vector3()
        self.cog_pose = Pose()
        self.cog_yaw = 0.0
        self.cog_start_pose =  Pose()
        self.cog_goal_pose = Pose()
        self.cog_before_pose = Pose()
        self.goal_vector = []

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
        self.sub_endeffector_goal_position = rospy.Subscriber("/set_goal", Vector3, self.cb_set_cog_goal_pose_mode0)
        self.sub_endeffector_goal_pose = rospy.Subscriber("/set_goal_pose", Pose, self.cb_set_cog_goal_pose_mode1)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher("/end_effector/pose", Pose, queue_size=10)
        self.sub_contact = rospy.Subscriber("/read_sensor", String, self.cb_detect_contact)
        self.timer_state = rospy.Timer(rospy.Duration(0.1), self.cb_publish_state)
        self.pub_state = rospy.Publisher("/my_flight_state", Int8, queue_size=10)
        self.pub_gpio = rospy.Publisher("/set_gpio", Empty, queue_size=10)

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
            return

    #コールバック:重心の位置姿勢更新(in:Pose)
    def cb_get_cog_current_pose(self, msg):
        self.cog_pose = msg.pose.pose
        angles = oq.quaternion_to_euler(self.cog_pose.orientation)
        self.cog_yaw = angles[2]

    #コールバック:重心の目標位置姿勢取得(in:Vector3),トピックの記録
    def cb_set_cog_goal_pose_mode0(self, msg):
        self.msg_register = msg
        self.direction_mode = 0
        #self.set_cog_goal_pose(msg)
        self.set_cog_goal_and_move(msg)
    #コールバック:重心の目標位置姿勢取得(in:Pose),トピックの記録
    def cb_set_cog_goal_pose_mode1(self, msg):
        self.msg_register_pose = msg
        self.direction_mode = 1
        #self.set_cog_goal_pose(msg)
        self.set_cog_goal_and_move(msg)
    #重心の目標位置姿勢取得(in:Vector3)
    def set_cog_goal_pose(self, msg):
        self.cog_start_pose = self.cog_pose
        if self.direction_mode == 0:
            self.set_direction_mode0(msg)
        elif self.direction_mode == 1:
            self.set_direction_mode1(msg)
            
        if self.distance > self.path_length*2:
            if self.direction_mode == 0:
                self.calc_pose_endeffector_to_cog(msg)
            elif self.direction_mode == 1:
                self.calc_pose_endeffector_to_cog(msg.position)
            self.partition()
            self.state = 1
            self.generate_cog_waypoint_list()
            self.state = 2
            self.publish_cog_waypoint()
            # if self.execution_mode == 0 and self.state == 2:
            #     self.state = 0
            #     if self.direction_mode == 0:
            #         self.set_cog_goal_pose(self.msg_register)
            #     elif self.direction_mode == 1:
            #         self.set_cog_goal_pose(self.msg_register_pose)
            # if self.execution_mode == 1:
            #     self.state = 3
            # self.waypoint_list.clear()
        else:
            self.simple_move()
        rospy.sleep(2.0)
        self.state = 4
        self.attach()
        rospy.sleep(5.0)
        self.detach()
    #重心の目標位置姿勢取得,移動
    def set_cog_goal_and_move(self, msg):
        self.cog_start_pose = self.cog_pose
        if self.direction_mode == 0:
            self.set_direction_mode0(msg)
            self.calc_pose_endeffector_to_cog(msg)
        elif self.direction_mode == 1:
            self.set_direction_mode1(msg)
            self.calc_pose_endeffector_to_cog(msg.position)

        self.state = 2
        duration = self.distance / self.mean_velocity
        rospy.loginfo("duration: %s", duration)
        self.approach(duration)
        rospy.sleep(duration + 2.0)
        self.state = 4
        self.attach()
        rospy.sleep(5.0)
        self.detach()

    #目標へのベクトル,角度,距離計算(in:Vector3)
    def set_direction_mode0(self, vec):
        self.direction.x = vec.x - self.endeffector_pose.position.x
        self.direction.y = vec.y - self.endeffector_pose.position.y
        self.direction_yaw = np.arctan2(self.direction.y, self.direction.x)
        self.direction_yaw_degree = self.direction_yaw * 180 / np.pi
        self.yaw_difference = self.direction_yaw - self.cog_yaw
        self.distance = np.sqrt(self.direction.x ** 2 + self.direction.y ** 2)
        rospy.loginfo("distance: %s", self.distance)
    def set_direction_mode1(self, pose):
        self.direction.x = pose.position.x - self.endeffector_pose.position.x
        self.direction.y = pose.position.y - self.endeffector_pose.position.y
        angles = oq.quaternion_to_euler(pose.orientation)
        self.direction_yaw = angles[2]
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
        angles = oq.quaternion_to_euler(self.cog_goal_pose.orientation)
        theta = angles[2]
        self.goal_vector = [np.cos(theta), np.sin(theta), 0]
        self.cog_direction.x = self.cog_goal_pose.position.x - self.cog_start_pose.position.x
        self.cog_direction.y = self.cog_goal_pose.position.y - self.cog_start_pose.position.y
        self.cog_direction.z = self.cog_goal_pose.position.z - self.cog_start_pose.position.z
        self.cog_before_pose.position.x = self.cog_goal_pose.position.x - (self.epsilon * self.goal_vector[0])
        self.cog_before_pose.position.y = self.cog_goal_pose.position.y - (self.epsilon * self.goal_vector[1])
        self.cog_before_pose.position.z = self.cog_goal_pose.position.z - (self.epsilon * self.goal_vector[2])
        self.cog_before_pose.orientation = self.cog_goal_pose.orientation
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
        if self.execution_mode == 0 and self.state == 12:
            self.state = 0
            if self.direction_mode == 0:
                self.set_cog_goal_pose(self.msg_register)
            elif self.direction_mode == 1:
                self.set_cog_goal_pose(self.msg_register_pose)
        if self.execution_mode == 1:
            self.state = 13
            
    #軌道の分割数決定
    def partition(self):
        n = int(self.distance // self.path_length)
        if n > 1:
            self.part = n
        rospy.loginfo("partition number: %s", self.part)

    #重心の経由点リストを計算
    def generate_cog_waypoint_list(self):
        if self.state != 1:
            return
        self.waypoint_list.clear()
        for i in range(1, self.part+1):
            waypoint = self.calc_cog_waypoint_pose(i)
            self.waypoint_list.append(waypoint)
            rospy.loginfo("waypoint: %s", waypoint.pose)
    #重心の経由点を計算(in:int)
    def calc_cog_waypoint_pose(self, i):
        if i > self.part:
            return
        ii = np.sqrt(float(i) / self.part)
        cog_waypoint_pose = PoseStamped()
        cog_waypoint_pose.pose.position.x = self.cog_start_pose.position.x + (self.cog_direction.x * ii)
        cog_waypoint_pose.pose.position.y = self.cog_start_pose.position.y + (self.cog_direction.y * ii)
        cog_waypoint_pose.pose.position.z = self.cog_start_pose.position.z
        yaw_degree = (self.cog_yaw + self.yaw_difference * ii) * 180 / np.pi
        cog_waypoint_pose.pose.orientation = oq.euler_to_quaternion(np.array([0, 0, yaw_degree]))
        return cog_waypoint_pose

    #中間点の送信
    def publish_cog_waypoint(self):
        for i in range(0, self.part):
            if self.state != 2:
                return
            self.pub.publish(self.waypoint_list[i])
            rospy.sleep(self.interval)

    #コールバック:接触検知による状態の更新
    def cb_detect_contact(self, msg):
        if self.state == 2 or self.state == 12 or self.state == 4:
            rospy.loginfo("contact detected")
            self.pub_gpio.publish()
            self.state += 1

    #状態を送信
    def cb_publish_state(self, event):
        self.pub_state.publish(self.state)

    #目標に接触
    def attach(self):
        print("---------------attach----------------\n")
        part = self.part_attach
        #angles = oq.quaternion_to_euler(self.cog_goal_pose.orientation)
        #theta = angles[2]
        #vector = [np.cos(theta), np.sin(theta), 0]
        rospy.loginfo("vector: %s", self.goal_vector)
        start_pose = self.cog_pose
        self.waypoint_list.clear()
        i = 0
        for i in range(0, part):
            waypoint_pose = PoseStamped()
            waypoint_pose.pose.position.x = start_pose.position.x + (i * self.epsilon * self.goal_vector[0])
            waypoint_pose.pose.position.y = start_pose.position.y + (i * self.epsilon * self.goal_vector[1])
            waypoint_pose.pose.position.z = start_pose.position.z
            waypoint_pose.pose.orientation = start_pose.orientation
            self.waypoint_list.append(waypoint_pose)
            #rospy.loginfo("waypoint: %s", waypoint_pose.pose)
        #print("--------------list---------------\n")
        #rospy.loginfo(self.waypoint_list)
        for i in range(1, part):
            if self.state != 4:
                return
            self.pub.publish(self.waypoint_list[i])
            rospy.sleep(self.interval)
        self.state += 1

    #目標から離脱
    def detach(self):
        if self.state != 5:
            return
        print("-----------detach------------\n")
        msg = PoseStamped()
        msg.header.stamp.secs = int(rospy.get_time() + 2)
        msg.pose = self.cog_before_pose
        self.pub.publish(msg)


    #目標の手前に接近
    def approach(self, duration):
        if self.state != 2:
            return
        msg = PoseStamped()
        msg.header.stamp.secs = int(rospy.get_time() + duration)
        msg.pose = self.cog_before_pose
        rospy.loginfo(msg)
        self.pub.publish(msg)
