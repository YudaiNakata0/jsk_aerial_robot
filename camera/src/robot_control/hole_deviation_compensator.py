#!/usr/bin/env python3
import threading
from collections import deque

import rospy
from aerial_robot_msgs.msg import SimpleFlightNav, Pid
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class HoleDeviationCompensator():
    """
    hole_target_tracker.pyが配信する基準位置からの偏差(/target/hole_deviation)を
    打ち消すように、y方向・z方向の速度指令を出す。z_center_control.pyと同じ
    PID制御・符号の考え方をベースにしているが、hole_deviationは既に基準位置
    からの誤差そのものであるため、目標中心との差分計算は不要。

    穴検出が途切れた場合は以下の順で復帰を試みる。
      1. HOLD   : 一瞬のロスト(ブレ・照明変化等)に備え、hold_duration秒だけ速度0で待機
      2. RETURN : 検出できていた間に記録した重心位置(odom)の履歴から、ロスト直前
                  recovery_lookback秒の位置を復帰目標とし、その位置へ速度指令で戻る
      3. GIVE_UP: recovery_timeout秒経っても再検出できなければ、その場で速度0を保持
    再検出されたら積分項等をリセットして通常のトラッキングに戻る。
    """

    STATE_TRACKING = "tracking"
    STATE_HOLD = "hold"
    STATE_RETURN = "return"
    STATE_GIVE_UP = "give_up"

    def __init__(self):
        self.lock = threading.Lock()
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.kp = rospy.get_param("~kp", 1e-04)
        self.ki = rospy.get_param("~ki", 1e-06)
        self.kd = rospy.get_param("~kd", 0.0)
        self.limit_i = rospy.get_param("~limit_i", 5e-02)
        self.limit_sum = rospy.get_param("~limit_sum", 0.02)

        self.kp_y = rospy.get_param("~kp_y", 1e-04)
        self.ki_y = rospy.get_param("~ki_y", 1e-06)
        self.kd_y = rospy.get_param("~kd_y", 0.0)
        self.limit_i_y = rospy.get_param("~limit_i_y", 5e-02)
        self.limit_sum_y = rospy.get_param("~limit_sum_y", 0.02)

        # ロスト時の復帰動作用パラメータ
        self.enable_recovery = rospy.get_param("~enable_recovery", True)
        self.hold_duration = rospy.get_param("~hold_duration", 0.5)            # [s]
        self.recovery_lookback = rospy.get_param("~recovery_lookback", 0.5)    # [s]
        self.recovery_timeout = rospy.get_param("~recovery_timeout", 10.0)     # [s]
        self.recovery_kp = rospy.get_param("~recovery_kp", 0.5)                # [1/s]
        self.recovery_limit_vel = rospy.get_param("~recovery_limit_vel", 0.03) # [m/s]
        self.recovery_tolerance = rospy.get_param("~recovery_tolerance", 0.01) # [m]
        self.history_duration = rospy.get_param("~history_duration", 5.0)      # [s]
        # validトピックがこの時間途絶えたらトラッカ停止とみなしロスト扱いにする
        self.valid_timeout = rospy.get_param("~valid_timeout", 0.5)            # [s]
        self.control_rate = rospy.get_param("~control_rate", 20.0)             # [Hz]

        self.state = self.STATE_TRACKING
        self.tracking_valid = False
        self.last_valid_msg_time = None
        self.lost_time = None
        self.recovery_goal = None
        self.cog_pos = None
        # 検出できていた間の(時刻, y, z)の履歴
        self.valid_history = deque()

        self.reset_pid()

    def reset_pid(self):
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.pre_error_x = 0.0
        self.pre_error_y = 0.0
        self.pre_time = rospy.get_time()

    def setup_ros(self):
        self.sub_deviation = rospy.Subscriber("/target/hole_deviation", Pose2D, self.deviation_callback)
        self.sub_valid = rospy.Subscriber("/target/hole_tracking_valid", Bool, self.valid_callback)
        self.sub_odom = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.odom_callback)
        self.pub_simple_nav = rospy.Publisher("/gimbalrotor/simple_nav", SimpleFlightNav, queue_size=1)
        self.pub_y_pid = rospy.Publisher("/y_pid_term", Pid, queue_size=1)
        self.pub_z_pid = rospy.Publisher("/z_pid_term", Pid, queue_size=1)
        # 検出が途絶えるとdeviationが配信されなくなるため、復帰動作はタイマで回す
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.timer_callback)

    # ---------------- callbacks ----------------

    def odom_callback(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            self.cog_pos = (p.y, p.z)

    def valid_callback(self, msg):
        with self.lock:
            self.last_valid_msg_time = rospy.get_time()
            if msg.data:
                self.on_tracking_found()
            else:
                self.on_tracking_lost()

    def deviation_callback(self, msg):
        with self.lock:
            if self.state != self.STATE_TRACKING or not self.tracking_valid:
                return
            self.record_valid_pose()
            self.compensate(msg)

    def timer_callback(self, _event):
        with self.lock:
            now = rospy.get_time()
            # トラッカ自体が落ちた場合もロスト扱いにする
            if self.tracking_valid and self.last_valid_msg_time is not None \
               and now - self.last_valid_msg_time > self.valid_timeout:
                rospy.logwarn("No tracking status for %.2f s." % (now - self.last_valid_msg_time))
                self.on_tracking_lost()

            if self.state == self.STATE_TRACKING:
                return
            self.recovery_step(now)

    # ---------------- state transitions ----------------

    def on_tracking_found(self):
        was_valid = self.tracking_valid
        self.tracking_valid = True
        if was_valid:
            return
        if self.state != self.STATE_TRACKING:
            rospy.loginfo("Hole tracking recovered (from %s)." % self.state)
        self.state = self.STATE_TRACKING
        self.recovery_goal = None
        # ロスト中の経過時間で微分・積分が跳ねないようにリセットする
        self.reset_pid()

    def on_tracking_lost(self):
        was_valid = self.tracking_valid
        self.tracking_valid = False
        if not was_valid:
            return
        rospy.logwarn("Hole tracking lost. Holding y/z velocity at 0.")
        self.lost_time = rospy.get_time()
        self.recovery_goal = self.select_recovery_goal(self.lost_time)
        self.state = self.STATE_HOLD
        self.reset_pid()
        self.publish_nav(0.0, 0.0)

    # ---------------- recovery ----------------

    def record_valid_pose(self):
        if self.cog_pos is None:
            return
        now = rospy.get_time()
        self.valid_history.append((now, self.cog_pos[0], self.cog_pos[1]))
        while self.valid_history and now - self.valid_history[0][0] > self.history_duration:
            self.valid_history.popleft()

    def select_recovery_goal(self, lost_time):
        # ロスト直前の位置は検出範囲の境界付近であることが多いため、
        # recovery_lookback秒だけ遡った、より確実に検出できていた位置を目標にする
        if not self.valid_history:
            return None
        target_time = lost_time - self.recovery_lookback
        goal = self.valid_history[-1]
        for sample in reversed(self.valid_history):
            goal = sample
            if sample[0] <= target_time:
                break
        return (goal[1], goal[2])

    def recovery_step(self, now):
        if self.state == self.STATE_GIVE_UP:
            self.publish_nav(0.0, 0.0)
            return

        elapsed = now - self.lost_time
        if elapsed > self.recovery_timeout:
            rospy.logerr("Could not re-detect the hole within %.1f s. Holding position." %
                         self.recovery_timeout)
            self.state = self.STATE_GIVE_UP
            self.publish_nav(0.0, 0.0)
            return

        if self.state == self.STATE_HOLD:
            if elapsed < self.hold_duration or not self.enable_recovery:
                self.publish_nav(0.0, 0.0)
                return
            if self.recovery_goal is None or self.cog_pos is None:
                rospy.logwarn_throttle(2.0, "No recorded pose to return to. Holding.")
                self.publish_nav(0.0, 0.0)
                return
            rospy.loginfo("Returning to last detectable pose (y=%.3f, z=%.3f)." % self.recovery_goal)
            self.state = self.STATE_RETURN

        # STATE_RETURN: 記録位置との差にP制御をかけ、y/zの速度指令として戻る
        # (通常のトラッキングと同じVEL_MODEのままにしてモード切替を避ける)
        err_y = self.recovery_goal[0] - self.cog_pos[0]
        err_z = self.recovery_goal[1] - self.cog_pos[1]
        if (err_y ** 2 + err_z ** 2) ** 0.5 < self.recovery_tolerance:
            rospy.logwarn_throttle(2.0, "Reached recorded pose but hole is still not detected.")
            self.publish_nav(0.0, 0.0)
            return
        v_y = self.clamp(self.recovery_kp * err_y, self.recovery_limit_vel)
        v_z = self.clamp(self.recovery_kp * err_z, self.recovery_limit_vel)
        self.publish_nav(v_y, v_z)

    # ---------------- tracking control ----------------

    def compensate(self, msg):
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
        i_y = self.clamp(-self.ki_y * self.integral_error_x, self.limit_i_y)
        d_y = -self.kd_y * (error_x - self.pre_error_x) / du
        v_y = self.clamp(p_y + i_y + d_y, self.limit_sum_y)

        p_z = -self.kp * error_y
        i_z = self.clamp(-self.ki * self.integral_error_y, self.limit_i)
        d_z = -self.kd * (error_y - self.pre_error_y) / du
        v_z = self.clamp(p_z + i_z + d_z, self.limit_sum)

        self.publish_nav(v_y, v_z)
        self.publish_pid_term(self.pub_y_pid, v_y, p_y, i_y, d_y)
        self.publish_pid_term(self.pub_z_pid, v_z, p_z, i_z, d_z)

        self.pre_error_x = error_x
        self.pre_error_y = error_y
        self.pre_time = current_time

    @staticmethod
    def clamp(value, limit):
        return max(min(value, limit), -limit)

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
