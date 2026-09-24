#!/usr/bin/env python3
import rospy
from aerial_robot_msgs.msg import SimpleFlightNav, Pid
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool


class HoleDeviationCompensator():
    """
    hole_target_tracker.pyが配信する基準位置からの偏差(/target/hole_deviation)を
    打ち消すように、y方向・z方向の速度指令を出す。z_center_control.pyと同じ
    PID制御・符号の考え方をベースにしているが、hole_deviationは既に基準位置
    からの誤差そのものであるため、目標中心との差分計算は不要。
    """

    def __init__(self):
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.kp = rospy.get_param("~kp", 3e-04)
        self.ki = rospy.get_param("~ki", 1e-06)
        self.kd = rospy.get_param("~kd", 0.0)
        self.limit_i = rospy.get_param("~limit_i", 5e-02)
        self.limit_sum = rospy.get_param("~limit_sum", 0.02)

        self.kp_y = rospy.get_param("~kp_y", 3e-04)
        self.ki_y = rospy.get_param("~ki_y", 1e-06)
        self.kd_y = rospy.get_param("~kd_y", 0.0)
        self.limit_i_y = rospy.get_param("~limit_i_y", 5e-02)
        self.limit_sum_y = rospy.get_param("~limit_sum_y", 0.02)

        self.tracking_valid = False
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.pre_error_x = 0.0
        self.pre_error_y = 0.0
        self.pre_time = rospy.get_time()

    def setup_ros(self):
        self.sub_deviation = rospy.Subscriber("/target/hole_deviation", Pose2D, self.deviation_callback)
        self.sub_valid = rospy.Subscriber("/target/hole_tracking_valid", Bool, self.valid_callback)
        self.pub_simple_nav = rospy.Publisher("/gimbalrotor/simple_nav", SimpleFlightNav, queue_size=1)
        self.pub_y_pid = rospy.Publisher("/y_pid_term", Pid, queue_size=1)
        self.pub_z_pid = rospy.Publisher("/z_pid_term", Pid, queue_size=1)

    def valid_callback(self, msg):
        was_valid = self.tracking_valid
        self.tracking_valid = msg.data
        if was_valid and not self.tracking_valid:
            rospy.logwarn("Hole tracking lost. Holding y/z velocity at 0.")
            self.publish_nav(0.0, 0.0)
            self.integral_error_x = 0.0
            self.integral_error_y = 0.0

    def deviation_callback(self, msg):
        if not self.tracking_valid:
            return

        current_time = rospy.get_time()
        du = current_time - self.pre_time
        if du <= 0.0:
            du = 1e-3

        # hole_target_tracker.pyがすでに基準位置からの偏差(dx, dy)として
        # 配信しているため、これをそのまま誤差として使う。画像のx/y軸
        # (右向き/下向きが正)と実機のy/z軸(横方向/上向きが正)は向きが
        # 逆になるため、z_center_control.pyと同様に誤差に負号を付けて
        # 速度指令にする。
        error_x = msg.x
        error_y = msg.y

        self.integral_error_x += error_x * du
        self.integral_error_y += error_y * du

        p_y = -self.kp_y * error_x
        i_y = -self.ki_y * self.integral_error_x
        i_y = max(min(i_y, self.limit_i_y), -self.limit_i_y)
        d_y = -self.kd_y * (error_x - self.pre_error_x) / du
        v_y = p_y + i_y + d_y
        v_y = max(min(v_y, self.limit_sum_y), -self.limit_sum_y)

        p_z = -self.kp * error_y
        i_z = -self.ki * self.integral_error_y
        i_z = max(min(i_z, self.limit_i), -self.limit_i)
        d_z = -self.kd * (error_y - self.pre_error_y) / du
        v_z = p_z + i_z + d_z
        v_z = max(min(v_z, self.limit_sum), -self.limit_sum)

        self.publish_nav(v_y, v_z)
        self.publish_pid_term(self.pub_y_pid, v_y, p_y, i_y, d_y)
        self.publish_pid_term(self.pub_z_pid, v_z, p_z, i_z, d_z)

        self.pre_error_x = error_x
        self.pre_error_y = error_y
        self.pre_time = current_time

    def publish_nav(self, v_y, v_z):
        msg = SimpleFlightNav()
        msg.y_control_mode = SimpleFlightNav.VEL_MODE
        msg.z_control_mode = SimpleFlightNav.VEL_MODE
        msg.vel_y = v_y
        msg.vel_z = v_z
        self.pub_simple_nav.publish(msg)

    def publish_pid_term(self, pub, total, p_term, i_term, d_term):
        msg = Pid()
        msg.total = [total]
        msg.p_term = [p_term]
        msg.i_term = [i_term]
        msg.d_term = [d_term]
        pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("hole_deviation_compensator")
    compensator = HoleDeviationCompensator()
    rospy.spin()
