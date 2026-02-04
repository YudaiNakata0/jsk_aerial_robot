#!/usr/bin/env python3
import rospy
from aerial_robot_msgs.msg import FlightNav, Pid, SimpleFlightNav
from geometry_msgs.msg import Vector3, Pose, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int8
import numpy as np

class ImageBaseApproach():
    def __init__(self):
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.endeffector_xi = rospy.get_param("~ee_x", 650)
        self.endeffector_yi = rospy.get_param("~ee_y", 410)
        self.target_xi = 0.0
        self.target_yi = 0.0
        
        self.kp = rospy.get_param("~kp", 1e-03)
        self.ki = rospy.get_param("~ki", 1e-06)
        self.kd = rospy.get_param("~kd", 1e-03)
        print(f"[ImageBaseApproach]endeffector coords [x:{self.endeffector_xi}, y:{self.endeffector_yi}], p gain:{self.kp}, i gain:{self.ki}, d gain:{self.kd}")
        self.integral_error_xi = 0.0
        self.integral_error_yi = 0.0
        self.limit_p = 0.1
        self.limit_i = 0.1
        self.limit_d = 0.1
        self.limit_sum_acc = 0.1
        self.mass = 3.18
        self.camera_angle_coef = np.cos(np.pi/6)
        
        self.cog_pose = Pose()
        self.cog_goal_pose = Pose()
        self.endeffector_pose = Pose()
        self.endeffector_goal_pose = Pose()
        
        self.is_cog_goal_record = False
        self.is_extruding = False
        self.cog_target_msg = PoseStamped()
        
        self.pre_error_xi = 0.0
        self.pre_error_yi = 0.0
        self.pre_time = rospy.get_time()
        
    def setup_ros(self):
        self.sub_target_circle = rospy.Subscriber("/target/2D_position", Vector3, self.callback)
        self.sub_endeffector = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.cb_record_cog_pose)
        self.sub_endeffector = rospy.Subscriber("/gimbalrotor/endeffector_pose", Pose, self.cb_record_endeffector_pose)
        self.pub_nav = rospy.Publisher("/gimbalrotor/uav/nav", FlightNav, queue_size=1)
        self.pub_simple_nav = rospy.Publisher("/gimbalrotor/simple_nav", SimpleFlightNav, queue_size=1)
        self.pub_cog = rospy.Publisher("/gimbalrotor/target_pose", PoseStamped, queue_size=1)
        self.pub_extrusion = rospy.Publisher("/set_gpio", Empty, queue_size=1)
        self.pub_y_pid = rospy.Publisher("/y_pid_term", Pid, queue_size=1)
        self.pub_z_pid = rospy.Publisher("/z_pid_term", Pid, queue_size=1)
        self.pub_y_control_mode = rospy.Publisher("/gimbalrotor/teleop_command/y_ctrl_mode", Int8, queue_size=1)
        self.pub_wrench = rospy.Publisher("/gimbalrotor/desire_wrench", WrenchStamped, queue_size=1)

    # カメラ画像を受け取ったとき
    def callback(self, msg):
        # 目標が検出されないとき
        if msg.z == 1:
            # yを位置制御モードに
            y_control_mode_msg = Int8()
            y_control_mode_msg.data = 0
            self.pub_y_control_mode.publish(y_control_mode_msg)
            if self.is_cog_goal_record:
                print("cannot catch target; move to recorded goal pose")
                self.publish_cog_target()
            return
        
        self.target_xi = msg.x
        self.target_yi = msg.y
        # 位置誤差（座標、距離）
        error_xi = self.target_xi - self.endeffector_xi
        error_yi = self.target_yi - self.endeffector_yi
        error_di2 = error_xi**2 + error_yi**2
        # 時間差
        current_time = rospy.get_time()
        du = current_time - self.pre_time
        # 誤差積算
        self.integral_error_xi += error_xi * du
        self.integral_error_yi += error_yi * du
        print(f"int_y: {self.integral_error_xi}, int_z: {self.integral_error_yi}")

        # 目標位置とエンドエフェクタ位置が近いとき
        if error_di2 < 400:
            print("approached successfully")
            # エンドエフェクタと重心の位置姿勢を記録
            self.cog_goal_pose = self.cog_pose
            self.endeffector_goal_pose = self.endeffector_pose
            self.cog_target_msg.pose = self.cog_goal_pose
            self.is_cog_goal_record = True
            # yを位置制御モードに
            y_control_mode_msg = Int8()
            y_control_mode_msg.data = 0
            self.pub_y_control_mode.publish(y_control_mode_msg)
            # 射出トリガを送信
            if not self.is_extruding:
                msg = Empty()
                self.pub_extrusion.publish(msg)
                self.is_extruding = True

        # 目標速度を計算　*カメラ画像内の軸と実際の軸は逆
        else:
            # yを速度制御モードに
            y_control_mode_msg = Int8()
            y_control_mode_msg.data = 1
            self.pub_y_control_mode.publish(y_control_mode_msg)    

            # 加速度入力計算
            p_y = -self.kp * error_xi
            p_z = -self.kp * error_yi
            i_y = -self.ki * self.integral_error_xi
            i_z = -self.ki * self.integral_error_yi
            print(f"i_y: {i_y}, i_z: {i_z}")
            i_y = max(min(i_y, self.limit_i), -self.limit_i)
            i_z = max(min(i_z, self.limit_i), -self.limit_i)
            d_y = -self.kd * (error_xi - self.pre_error_xi) / du
            d_z = -self.kd * (error_yi - self.pre_error_yi) / du
            a_y = p_y + i_y + d_y
            a_z = p_z + i_z + d_z
            a_y = max(min(a_y, self.limit_sum_acc), -self.limit_sum_acc)
            a_z = max(min(a_z, self.limit_sum_acc), -self.limit_sum_acc)
            w_y = a_y * self.mass
            w_z = a_z * self.mass
            self.publish_acc_to_desire_wrench(w_y, w_z)
            # self.publish_acc(v_y, v_z)
            # pid各項の値を記録
            pid_msg_y = Pid()
            pid_msg_y.total = [v_y]
            pid_msg_y.p_term = [p_y]
            pid_msg_y.i_term = [i_y]
            pid_msg_y.d_term = [d_y]
            self.pub_y_pid.publish(pid_msg_y)
            pid_msg_z = Pid()
            pid_msg_z.total = [v_z]
            pid_msg_z.p_term = [p_z]
            pid_msg_z.i_term = [i_z]
            pid_msg_z.d_term = [d_z]
            self.pub_z_pid.publish(pid_msg_z)

        # 位置誤差を記録
        self.pre_error_xi = error_xi
        self.pre_error_yi = error_yi

    # ROSトピック送信（速度制御モード）
    def publish(self, v_y, v_z):
        pub_msg = SimpleFlightNav()
        pub_msg.x_control_mode = SimpleFlightNav.VEL_MODE
        pub_msg.y_control_mode = SimpleFlightNav.POS_VEL_MODE
        pub_msg.z_control_mode = SimpleFlightNav.VEL_MODE
        pub_msg.vel_y = v_y
        pub_msg.vel_z = v_z
        rospy.loginfo("publish message to uav/nav: %s, %s", v_y, v_z)
        self.pub_simple_nav.publish(pub_msg)

    # ROSトピック送信（加速度制御モード）
    def publish_acc(self, a_y, a_z):
        pub_msg = SimpleFlightNav()
        pub_msg.x_control_mode = SimpleFlightNav.ACC_MODE
        pub_msg.y_control_mode = SimpleFlightNav.ACC_MODE
        pub_msg.z_control_mode = SimpleFlightNav.ACC_MODE
        pub_msg.acc_y = a_y
        pub_msg.acc_z = a_z
        rospy.loginfo("publish message to simple_nav: %s, %s", a_y, a_z)
        self.pub_simple_nav.publish(pub_msg)

    # ROSトピック送信（加速度、力次元）
    def publish_acc_to_desire_wrench(self, w_y, w_z):
        pub_msg = WrenchStamped()
        pub_msg.wrench.force.x = 1.0
        pub_msg.wrench.force.y = w_y
        pub_msg.wrench.force.z = w_z
        rospy.loginfo("publish message to desire_wrench: %s, %s", w_y, w_z)
        self.pub_wrench.publish(pub_msg)

        
    # 重心位置姿勢取得
    def cb_record_cog_pose(self, msg):
        self.cog_pose = msg.pose.pose

    # エンドエフェクタ位置姿勢取得
    def cb_record_endeffector_pose(self, msg):
        self.endeffector_pose = msg

    # 記録した重心の位置に移動
    def publish_cog_target(self):
        if self.is_cog_goal_record:
            if self.cog_target_msg.pose.position.z > 0.8:
                self.pub_cog.publish(self.cog_target_msg)
                print("published cog pose")

if __name__ == "__main__":
    rospy.init_node("navigation_node")
    navigator = ImageBaseApproach()
    r = rospy.Rate(0.5)
    # while not rospy.is_shutdown():
    #     navigator.publish_cog_target()
    #     r.sleep()
    rospy.spin()
