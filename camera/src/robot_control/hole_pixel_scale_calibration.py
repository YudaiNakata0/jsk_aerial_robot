#!/usr/bin/env python3
import datetime
import os
import threading

import numpy as np
import rospy
import yaml
from aerial_robot_msgs.msg import SimpleFlightNav
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class HolePixelScaleCalibration():
    """
    hole_world_estimate_compensator.py で使う「画素偏差 -> 機体位置」の
    換算係数 scale [m/px] (= 壁までの距離 D / 焦点距離 f) を求めるための
    キャリブレーション飛行を行う。

    手順:
      1. hole_target_tracker.py で穴をトラッキングできている状態でホバリングさせる
         (hole_*_compensator.py は simple_nav が衝突するため起動しないこと)
      2. 本スクリプトを起動し、Enterで開始
      3. 開始時の位置を原点として y, z をそれぞれ offsets の各点へ位置指令で動かし、
         各点で静定後に odom 位置と画素偏差を平均する
      4. 画素偏差 d と位置 p の直線 d = p / scale + c を当てはめ、scale を YAML に保存
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.setup_parameters()
        self.setup_ros()

    def setup_parameters(self):
        self.offsets = rospy.get_param("~offsets", [0.0, 0.02, 0.04, 0.02, 0.0, -0.02, -0.04, -0.02, 0.0])  # [m]
        self.max_offset = rospy.get_param("~max_offset", 0.06)            # [m] 安全のための上限
        self.settle_timeout = rospy.get_param("~settle_timeout", 6.0)     # [s]
        self.settle_min_time = rospy.get_param("~settle_min_time", 1.5)   # [s]
        self.settle_pos_tol = rospy.get_param("~settle_pos_tol", 0.005)   # [m]
        self.settle_vel_tol = rospy.get_param("~settle_vel_tol", 0.01)    # [m/s]
        self.sample_time = rospy.get_param("~sample_time", 1.0)           # [s]
        self.min_valid_ratio = rospy.get_param("~min_valid_ratio", 0.8)
        self.min_points = rospy.get_param("~min_points", 4)
        self.wait_for_enter = rospy.get_param("~wait_for_enter", True)
        default_output = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "..", "config", "hole_pixel_scale.yaml")
        self.output_path = os.path.abspath(os.path.expanduser(rospy.get_param("~output_path", default_output)))
        self.rate = rospy.Rate(rospy.get_param("~rate", 20.0))

        for offset in self.offsets:
            if abs(offset) > self.max_offset:
                raise ValueError("offset %.3f exceeds max_offset %.3f" % (offset, self.max_offset))

        self.cog_pos = None
        self.cog_vel = None
        self.tracking_valid = False
        self.deviation = None
        self.deviation_time = None
        self.origin = None

    def setup_ros(self):
        self.sub_deviation = rospy.Subscriber("/target/hole_deviation", Pose2D, self.deviation_callback)
        self.sub_valid = rospy.Subscriber("/target/hole_tracking_valid", Bool, self.valid_callback)
        self.sub_odom = rospy.Subscriber("/gimbalrotor/uav/cog/odom", Odometry, self.odom_callback)
        self.pub_simple_nav = rospy.Publisher("/gimbalrotor/simple_nav", SimpleFlightNav, queue_size=1)

    # ---------------- callbacks ----------------

    def odom_callback(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            self.cog_pos = (p.y, p.z)
            self.cog_vel = (v.y, v.z)

    def valid_callback(self, msg):
        with self.lock:
            self.tracking_valid = msg.data

    def deviation_callback(self, msg):
        with self.lock:
            self.deviation = (msg.x, msg.y)
            self.deviation_time = rospy.get_time()

    # ---------------- motion ----------------

    def publish_target(self, y, z):
        msg = SimpleFlightNav()
        msg.y_control_mode = SimpleFlightNav.POS_MODE
        msg.z_control_mode = SimpleFlightNav.POS_MODE
        msg.pos_y = y
        msg.pos_z = z
        self.pub_simple_nav.publish(msg)

    def move_and_settle(self, y, z):
        # 位置指令を一定周期で送りつつ、目標付近で速度が小さくなるまで待つ
        start = rospy.get_time()
        self.publish_target(y, z)
        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start
            with self.lock:
                pos, vel = self.cog_pos, self.cog_vel
            err = max(abs(pos[0] - y), abs(pos[1] - z))
            speed = max(abs(vel[0]), abs(vel[1]))
            if elapsed > self.settle_min_time and err < self.settle_pos_tol and speed < self.settle_vel_tol:
                return True
            if elapsed > self.settle_timeout:
                rospy.logwarn("Settle timeout at (y=%.3f, z=%.3f): err=%.4f m, speed=%.4f m/s" %
                              (y, z, err, speed))
                # 位置誤差が残っても、実際の odom 位置で記録するので計測は続行する
                return True
            self.publish_target(y, z)
            self.rate.sleep()
        return False

    def sample(self):
        # sample_time 秒間、トラッキングが有効なフレームの位置と偏差を平均する
        positions, deviations = [], []
        total = 0
        last_dev_time = None
        start = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start < self.sample_time:
            with self.lock:
                valid, dev, dev_time, pos = self.tracking_valid, self.deviation, self.deviation_time, self.cog_pos
            if dev_time is not None and dev_time != last_dev_time:
                last_dev_time = dev_time
                total += 1
                if valid:
                    positions.append(pos)
                    deviations.append(dev)
            self.rate.sleep()
        if total == 0 or len(deviations) < self.min_valid_ratio * total:
            rospy.logwarn("Tracking was not valid enough during sampling (%d/%d). Skip this point." %
                          (len(deviations), total))
            return None
        return np.mean(positions, axis=0), np.mean(deviations, axis=0), np.std(deviations, axis=0)

    def sweep(self, axis):
        # axis: 0 -> y方向に動かす, 1 -> z方向に動かす
        records = []
        name = "y" if axis == 0 else "z"
        for offset in self.offsets:
            if rospy.is_shutdown():
                break
            target = list(self.origin)
            target[axis] += offset
            rospy.loginfo("[%s sweep] move to offset %+.3f m" % (name, offset))
            self.move_and_settle(target[0], target[1])
            result = self.sample()
            if result is None:
                continue
            pos, dev, dev_std = result
            rospy.loginfo("[%s sweep]   pos=(%.4f, %.4f) dev=(%.1f, %.1f) std=(%.1f, %.1f)" %
                          (name, pos[0], pos[1], dev[0], dev[1], dev_std[0], dev_std[1]))
            records.append((pos, dev))
        return records

    # ---------------- fitting ----------------

    def fit(self, records, axis):
        name = "y" if axis == 0 else "z"
        if len(records) < self.min_points:
            rospy.logerr("[%s] Not enough valid points (%d < %d)." % (name, len(records), self.min_points))
            return None
        pos = np.array([r[0][axis] for r in records])
        dev_main = np.array([r[1][axis] for r in records])       # y移動ならu, z移動ならv
        dev_cross = np.array([r[1][1 - axis] for r in records])  # もう一方の画素軸
        if np.ptp(pos) < 1e-3:
            rospy.logerr("[%s] Robot did not move enough (range %.4f m)." % (name, np.ptp(pos)))
            return None

        # d = slope * p + c  (slope [px/m])
        slope, intercept = np.polyfit(pos, dev_main, 1)
        residual = dev_main - (slope * pos + intercept)
        ss_tot = np.sum((dev_main - np.mean(dev_main)) ** 2)
        r2 = 1.0 - np.sum(residual ** 2) / ss_tot if ss_tot > 0 else 0.0
        cross_slope = np.polyfit(pos, dev_cross, 1)[0]

        # 画像軸と機体軸は逆向きという前提 (hole_deviation_compensator.pyの
        # v = -k*d が正しく働く向き) では slope は正になる
        if slope <= 0.0:
            rospy.logerr("[%s] Slope is not positive (%.1f px/m). The sign convention of "
                         "the compensator would be wrong. Check the camera/axis setup." % (name, slope))
            return None
        scale = 1.0 / slope
        result = {
            "scale": float(scale),
            "slope_px_per_m": float(slope),
            "r2": float(r2),
            "cross_coupling": float(cross_slope / slope),
            "max_residual_px": float(np.max(np.abs(residual))),
            "num_points": len(records),
            "pos_range_m": float(np.ptp(pos)),
        }
        rospy.loginfo("[%s] scale=%.3e m/px (slope=%.1f px/m), R2=%.4f, cross=%.3f, max residual=%.1f px" %
                      (name, scale, slope, r2, result["cross_coupling"], result["max_residual_px"]))
        if r2 < 0.95:
            rospy.logwarn("[%s] R2 is low. Tracking noise or nonlinearity (lens distortion) may be large." % name)
        if abs(result["cross_coupling"]) > 0.2:
            rospy.logwarn("[%s] Large cross coupling. Yaw may not be aligned with the wall." % name)
        return result

    def save(self, result_y, result_z):
        data = {
            "initial_scale_y": result_y["scale"],
            "initial_scale_z": result_z["scale"],
            "calibration": {
                "date": datetime.datetime.now().isoformat(timespec="seconds"),
                "origin_y": float(self.origin[0]),
                "origin_z": float(self.origin[1]),
                "y": result_y,
                "z": result_z,
            },
        }
        directory = os.path.dirname(self.output_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(self.output_path, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
        rospy.loginfo("Saved calibration result to %s" % self.output_path)

    # ---------------- main ----------------

    def wait_ready(self):
        rospy.loginfo("Waiting for odom and valid hole tracking...")
        while not rospy.is_shutdown():
            with self.lock:
                ready = self.cog_pos is not None and self.tracking_valid and self.deviation is not None
            if ready:
                return True
            self.rate.sleep()
        return False

    def run(self):
        if not self.wait_ready():
            return
        if self.wait_for_enter:
            input("Hovering in front of the hole with tracking valid? Press Enter to start calibration flight.")
        with self.lock:
            self.origin = self.cog_pos
        rospy.loginfo("Origin: y=%.4f, z=%.4f. offsets=%s" % (self.origin[0], self.origin[1], self.offsets))

        try:
            records_y = self.sweep(0)
            self.move_and_settle(self.origin[0], self.origin[1])
            records_z = self.sweep(1)
        finally:
            if not rospy.is_shutdown():
                rospy.loginfo("Returning to origin.")
                self.move_and_settle(self.origin[0], self.origin[1])

        result_y = self.fit(records_y, 0)
        result_z = self.fit(records_z, 1)
        if result_y is None or result_z is None:
            rospy.logerr("Calibration failed. Nothing is saved.")
            return
        self.save(result_y, result_z)


if __name__ == "__main__":
    rospy.init_node("hole_pixel_scale_calibration")
    calibration = HolePixelScaleCalibration()
    calibration.run()
