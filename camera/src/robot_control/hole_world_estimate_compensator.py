#!/usr/bin/env python3
import os
import threading
from collections import deque

import rospy
import yaml
from aerial_robot_msgs.msg import SimpleFlightNav, Pid
from geometry_msgs.msg import Pose2D, PointStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class AxisScaleEstimator():
    """
    1軸分の「画素偏差 -> 機体位置」換算係数 a [m/px] を推定する。
    穴は壁に固定されているため、穴を基準点に合わせられる機体位置
        p* = p + a * d      (p: 機体位置 [m], d: 画素偏差 [px])
    は一定である。よって機体が動いたときの変位の組(Δp, Δd)は
        Δp + a * Δd = 0
    を満たすので、忘却係数付きの最小二乗で a = -Σ(Δp Δd) / Σ(Δd^2) を求める。
    初期値 a0 には prior_weight の重みを持たせ、動きが少ない間はそちらに寄せる。
    """

    def __init__(self, initial_scale, prior_weight, forgetting, min_scale, max_scale, fixed=False):
        # 画像の軸と機体の軸は逆向きなので a は負になる (a = -D/f)
        self.a0 = -initial_scale
        # fixed=True のときは initial_scale をそのまま使い、オンライン更新しない
        self.fixed = fixed
        self.prior_weight = prior_weight
        self.forgetting = forgetting
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.reset()

    def reset(self):
        self.s_dd = self.prior_weight
        self.s_pd = -self.a0 * self.prior_weight
        self.num_updates = 0

    def update(self, dp, dd):
        if self.fixed:
            return
        self.s_dd = self.forgetting * self.s_dd + dd * dd
        self.s_pd = self.forgetting * self.s_pd + dp * dd
        self.num_updates += 1

    def is_reliable(self, min_updates):
        return self.fixed or self.num_updates >= min_updates

    def scale(self):
        # 符号が反転したり極端な値になったりしないよう範囲を制限する
        scale = self.s_pd / self.s_dd
        return max(min(scale, self.max_scale), self.min_scale)

    def a(self):
        return -self.scale()


class HoleWorldEstimateCompensator():
    """
    hole_deviation_compensator.pyと同じ画素偏差フィードバックで穴を追従しつつ、
    検出中は「穴を基準点に合わせられる機体のworld位置 p*(y, z)」を推定し続ける。
    換算係数(m/px)はカメラキャリブレーションや壁距離を使わず、機体の移動量
    (odom)と画素偏差の変化量の関係からオンラインで推定する(AxisScaleEstimator)。

    穴検出が途切れた場合:
      1. HOLD   : hold_duration秒だけ速度0で待機
      2. RETURN : p*が十分推定できていればp*へ、そうでなければ検出できていた
                  位置の履歴(ロスト直前recovery_lookback秒前)へ速度指令で戻る
      3. GIVE_UP: recovery_timeout秒経っても再検出できなければ速度0を保持

    前提: 壁の法線がworldのx軸にほぼ一致し(yaw≈0)、壁までの距離が大きく
    変わらないこと。距離が変わると換算係数も変わるため、忘却係数で追従させる。
    hole_pixel_scale_calibration.py で事前に求めた係数を ~scale_file で読み込み、
    ~use_online_scale:=false とすれば係数を固定して使う。
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

        # 換算係数の推定用パラメータ
        # initial_scale: 概算の D/f [m/px] (例: 壁まで0.5m, f=800px なら 6.25e-4)
        initial_scale_y = rospy.get_param("~initial_scale_y", 6e-04)
        initial_scale_z = rospy.get_param("~initial_scale_z", 6e-04)
        # hole_pixel_scale_calibration.py の出力YAMLを指定すると initial_scale_* を上書きする
        scale_file = rospy.get_param("~scale_file", "")
        if scale_file:
            initial_scale_y, initial_scale_z = self.load_scale_file(scale_file)
        # false のときは initial_scale_* (キャリブレーション値) を固定で使う
        use_online_scale = rospy.get_param("~use_online_scale", True)
        prior_weight = rospy.get_param("~prior_weight", 400.0)      # [px^2] 初期値の重み
        forgetting = rospy.get_param("~forgetting", 0.98)
        min_scale = rospy.get_param("~min_scale", 1e-05)            # [m/px]
        max_scale = rospy.get_param("~max_scale", 1e-02)            # [m/px]
        fixed = not use_online_scale
        self.scale_est_y = AxisScaleEstimator(initial_scale_y, prior_weight, forgetting, min_scale, max_scale, fixed)
        self.scale_est_z = AxisScaleEstimator(initial_scale_z, prior_weight, forgetting, min_scale, max_scale, fixed)
        rospy.loginfo("pixel scale: y=%.3e, z=%.3e m/px (%s)" %
                      (initial_scale_y, initial_scale_z, "online" if use_online_scale else "fixed"))
        # 差分を取る間隔。短すぎるとノイズや画像遅延の影響が大きくなる
        self.pair_interval = rospy.get_param("~pair_interval", 0.3)         # [s]
        # これより画素変化が小さい区間は係数更新に使わない(ノイズ対策)
        self.min_pixel_step = rospy.get_param("~min_pixel_step", 5.0)       # [px]
        # 1フレームでこれ以上偏差が跳んだら基準点の再設定等とみなし推定をリセット
        self.max_pixel_jump = rospy.get_param("~max_pixel_jump", 80.0)      # [px]
        # 画像取得から偏差配信までの遅延。この分だけ過去のodomと対応させる
        self.image_delay = rospy.get_param("~image_delay", 0.05)            # [s]

        # p* 推定用パラメータ
        self.goal_filter_alpha = rospy.get_param("~goal_filter_alpha", 0.1)
        self.goal_outlier_thresh = rospy.get_param("~goal_outlier_thresh", 0.05)  # [m]
        self.goal_min_samples = rospy.get_param("~goal_min_samples", 30)
        # 換算係数が初期値以外の情報で何回更新されたら p* を信用するか
        self.scale_min_updates = rospy.get_param("~scale_min_updates", 3)

        # ロスト時の復帰動作用パラメータ
        self.enable_recovery = rospy.get_param("~enable_recovery", True)
        self.hold_duration = rospy.get_param("~hold_duration", 0.5)            # [s]
        self.recovery_lookback = rospy.get_param("~recovery_lookback", 0.5)    # [s]
        self.recovery_timeout = rospy.get_param("~recovery_timeout", 10.0)     # [s]
        self.recovery_kp = rospy.get_param("~recovery_kp", 0.5)                # [1/s]
        self.recovery_limit_vel = rospy.get_param("~recovery_limit_vel", 0.03) # [m/s]
        self.recovery_tolerance = rospy.get_param("~recovery_tolerance", 0.01) # [m]
        self.history_duration = rospy.get_param("~history_duration", 5.0)      # [s]
        self.valid_timeout = rospy.get_param("~valid_timeout", 0.5)            # [s]
        self.control_rate = rospy.get_param("~control_rate", 20.0)             # [Hz]

        self.state = self.STATE_TRACKING
        self.tracking_valid = False
        self.last_valid_msg_time = None
        self.lost_time = None
        self.recovery_goal = None
        self.recovery_goal_source = None
        self.cog_pos = None
        # odomの(時刻, y, z)履歴。画像遅延分だけ過去の位置を引くのに使う
        self.odom_history = deque()
        # 検出できていた間の(時刻, y, z)の履歴(p*が使えないときの復帰先)
        self.valid_history = deque()

        self.reset_pid()
        self.reset_estimate()

    @staticmethod
    def load_scale_file(path):
        path = os.path.expanduser(path)
        with open(path) as f:
            data = yaml.safe_load(f)
        rospy.loginfo("Loaded pixel scale from %s (calibrated at %s)" %
                      (path, data.get("calibration", {}).get("date", "unknown")))
        return float(data["initial_scale_y"]), float(data["initial_scale_z"])

    def reset_pid(self):
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.pre_error_x = 0.0
        self.pre_error_y = 0.0
        self.pre_time = rospy.get_time()

    def reset_estimate(self):
        self.goal_y = None
        self.goal_z = None
        self.goal_samples = 0
        self.reset_pair()

    def reset_pair(self):
        # 差分を取る基準のサンプル (時刻, y, z, du, dv)
        self.pair_anchor = None
        self.last_deviation = None

    def setup_ros(self):
        self.sub_deviation = rospy.Subscriber("/target/hole_deviation", Pose2D, self.deviation_callback)
        self.sub_valid = rospy.Subscriber("/target/hole_tracking_valid", Bool, self.valid_callback)
        self.sub_odom = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.odom_callback)
        self.pub_simple_nav = rospy.Publisher("/gimbalrotor/simple_nav", SimpleFlightNav, queue_size=1)
        self.pub_y_pid = rospy.Publisher("/y_pid_term", Pid, queue_size=1)
        self.pub_z_pid = rospy.Publisher("/z_pid_term", Pid, queue_size=1)
        # 推定結果の確認用。機体を動かしても hole_goal がほぼ一定なら推定は正しい
        self.pub_goal = rospy.Publisher("/hole_world_estimate/goal", PointStamped, queue_size=1)
        self.pub_scale = rospy.Publisher("/hole_world_estimate/scale", Vector3, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.timer_callback)

    # ---------------- callbacks ----------------

    def odom_callback(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            now = rospy.get_time()
            self.cog_pos = (p.y, p.z)
            self.odom_history.append((now, p.y, p.z))
            while self.odom_history and now - self.odom_history[0][0] > 1.0:
                self.odom_history.popleft()

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
            self.update_estimate(msg.x, msg.y)
            self.compensate(msg)

    def timer_callback(self, _event):
        with self.lock:
            now = rospy.get_time()
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
        self.reset_pid()
        # ロスト前後の変位を係数推定に使わないよう差分の基準を取り直す
        self.reset_pair()

    def on_tracking_lost(self):
        was_valid = self.tracking_valid
        self.tracking_valid = False
        if not was_valid:
            return
        rospy.logwarn("Hole tracking lost. Holding y/z velocity at 0.")
        self.lost_time = rospy.get_time()
        self.recovery_goal, self.recovery_goal_source = self.select_recovery_goal(self.lost_time)
        self.state = self.STATE_HOLD
        self.reset_pid()
        self.reset_pair()
        self.publish_nav(0.0, 0.0)

    # ---------------- estimation ----------------

    def delayed_pos(self):
        # 画像遅延を考慮し、image_delay秒前のodom位置を返す
        if not self.odom_history:
            return None
        target_time = rospy.get_time() - self.image_delay
        for sample in reversed(self.odom_history):
            if sample[0] <= target_time:
                return (sample[1], sample[2])
        return (self.odom_history[0][1], self.odom_history[0][2])

    def update_estimate(self, du, dv):
        pos = self.delayed_pos()
        if pos is None:
            return
        now = rospy.get_time()

        # 基準点の再設定('c'キー)などで偏差が不連続に跳んだ場合、
        # p* も換算係数用の差分も意味を失うので推定をやり直す
        if self.last_deviation is not None:
            jump = max(abs(du - self.last_deviation[0]), abs(dv - self.last_deviation[1]))
            if jump > self.max_pixel_jump:
                rospy.logwarn("Deviation jumped by %.1f px. Resetting hole goal estimate." % jump)
                self.reset_estimate()
        self.last_deviation = (du, dv)

        # --- 換算係数の更新 ---
        if self.pair_anchor is None:
            self.pair_anchor = (now, pos[0], pos[1], du, dv)
        elif now - self.pair_anchor[0] >= self.pair_interval:
            dp_y = pos[0] - self.pair_anchor[1]
            dp_z = pos[1] - self.pair_anchor[2]
            dd_u = du - self.pair_anchor[3]
            dd_v = dv - self.pair_anchor[4]
            if abs(dd_u) >= self.min_pixel_step:
                self.scale_est_y.update(dp_y, dd_u)
            if abs(dd_v) >= self.min_pixel_step:
                self.scale_est_z.update(dp_z, dd_v)
            self.pair_anchor = (now, pos[0], pos[1], du, dv)

        # --- p* の更新 ---
        goal_y = pos[0] + self.scale_est_y.a() * du
        goal_z = pos[1] + self.scale_est_z.a() * dv
        if self.goal_y is None:
            self.goal_y, self.goal_z = goal_y, goal_z
            self.goal_samples = 1
        else:
            err = max(abs(goal_y - self.goal_y), abs(goal_z - self.goal_z))
            if self.goal_samples >= self.goal_min_samples and err > self.goal_outlier_thresh:
                rospy.logwarn_throttle(1.0, "Hole goal outlier rejected (%.3f m)." % err)
            else:
                alpha = self.goal_filter_alpha
                self.goal_y += alpha * (goal_y - self.goal_y)
                self.goal_z += alpha * (goal_z - self.goal_z)
                self.goal_samples += 1

        self.publish_estimate()

    def goal_is_reliable(self):
        return self.goal_y is not None \
            and self.goal_samples >= self.goal_min_samples \
            and self.scale_est_y.is_reliable(self.scale_min_updates) \
            and self.scale_est_z.is_reliable(self.scale_min_updates)

    # ---------------- recovery ----------------

    def record_valid_pose(self):
        if self.cog_pos is None:
            return
        now = rospy.get_time()
        self.valid_history.append((now, self.cog_pos[0], self.cog_pos[1]))
        while self.valid_history and now - self.valid_history[0][0] > self.history_duration:
            self.valid_history.popleft()

    def select_recovery_goal(self, lost_time):
        if self.goal_is_reliable():
            return (self.goal_y, self.goal_z), "estimated hole goal"

        # p* が未収束なら、ロスト直前 recovery_lookback 秒の位置へ戻る
        if not self.valid_history:
            return None, None
        target_time = lost_time - self.recovery_lookback
        goal = self.valid_history[-1]
        for sample in reversed(self.valid_history):
            goal = sample
            if sample[0] <= target_time:
                break
        return (goal[1], goal[2]), "last detectable pose"

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
            rospy.loginfo("Returning to %s (y=%.3f, z=%.3f)." %
                          (self.recovery_goal_source, self.recovery_goal[0], self.recovery_goal[1]))
            self.state = self.STATE_RETURN

        err_y = self.recovery_goal[0] - self.cog_pos[0]
        err_z = self.recovery_goal[1] - self.cog_pos[1]
        if (err_y ** 2 + err_z ** 2) ** 0.5 < self.recovery_tolerance:
            rospy.logwarn_throttle(2.0, "Reached %s but hole is still not detected." %
                                   self.recovery_goal_source)
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

        # 画像のx/y軸(右向き/下向きが正)と実機のy/z軸は向きが逆になるため、
        # 誤差に負号を付けて速度指令にする (hole_deviation_compensator.pyと同じ)
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

    def publish_estimate(self):
        goal_msg = PointStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "world"
        goal_msg.point.x = float(self.goal_samples)  # xは未使用のためサンプル数を入れておく
        goal_msg.point.y = self.goal_y
        goal_msg.point.z = self.goal_z
        self.pub_goal.publish(goal_msg)
        self.pub_scale.publish(Vector3(x=self.scale_est_y.scale(), y=self.scale_est_z.scale(),
                                       z=float(min(self.scale_est_y.num_updates,
                                                   self.scale_est_z.num_updates))))

    def publish_pid_term(self, pub, total, p_term, i_term, d_term):
        msg = Pid()
        msg.total = [total]
        msg.p_term = [p_term]
        msg.i_term = [i_term]
        msg.d_term = [d_term]
        pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("hole_world_estimate_compensator")
    compensator = HoleWorldEstimateCompensator()
    rospy.spin()
