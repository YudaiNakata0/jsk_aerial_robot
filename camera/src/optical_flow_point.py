#!/usr/bin/env python3
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

class OpticalFlowTracker:
    def __init__(self, topic="/usb_cam/image_raw", path=None):
        self.bridge = CvBridge()
        self.topic = topic

        # Lucas-Kanade法パラメータ
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS |
                                        cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # 状態管理
        self.prev_gray = None
        self.prev_points = None
        self.mask = None
        self.is_window_created = False

        self.click_points = []

        rospy.Subscriber(self.topic, Image, self.callback)
        self.pub_result = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)
        self.mask_invalid_part = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if self.mask_invalid_part is None:
            raise RuntimeError("cannot read mask image.")

    def mouse_callback(self, event, x, y, flags, param):
        """マウスクリックで特徴点を追加"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mask_invalid_part[y, x] > 0:
                rospy.loginfo("invalid point.")
                return
            self.click_points.append([[x, y]])
            rospy.loginfo(f"クリック追加: ({x}, {y})")
            self.prev_points = np.array(self.click_points, dtype=np.float32)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 最初のフレーム時にウィンドウ作成＆マウスコールバック設定
        if not self.is_window_created:
            cv2.namedWindow("Optical Flow (ROS)")
            cv2.setMouseCallback("Optical Flow (ROS)", self.mouse_callback)
            self.mask = np.zeros_like(frame)
            self.is_window_created = True
            self.prev_gray = gray
            return

        if self.prev_points is None or len(self.prev_points) == 0:
            cv2.imshow("Optical Flow (ROS)", frame)
            cv2.waitKey(1)
            self.prev_gray = gray
            return

        # Optical Flow計算
        next_points, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None, **self.lk_params)

        if next_points is not None:
            good_new = next_points[status == 1]
            good_old = self.prev_points[status == 1]

            x = good_new[:, 0].astype(int)
            y = good_new[:, 1].astype(int)
            is_invalid = self.mask_invalid_part[y, x] > 0
            good_new = good_new[~is_invalid]
            good_old = good_old[~is_invalid]

            for new, old in zip(good_new, good_old):
                a, b = new.ravel()
                c, d = old.ravel()
                self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

            output = cv2.add(frame, self.mask)
            cv2.imshow("Optical Flow (ROS)", output)

            if len(good_new) == 0:
                rospy.loginfo("lost points.")
                self.prev_points = None
            else:
                self.prev_points = good_new.reshape(-1, 1, 2)
                print(self.prev_points)
                msg = Vector3()
                msg.x = self.prev_points[0][0][0]
                msg.y = self.prev_points[0][0][1]
                self.pub_result.publish(msg)
        else:
            cv2.imshow("Optical Flow (ROS)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            rospy.signal_shutdown("User exited")
        elif key == ord('r'):
            rospy.loginfo("特徴点リセット")
            self.click_points = []
            self.prev_points = None
            self.mask = np.zeros_like(frame)

        self.prev_gray = gray

    def run(self):
        rospy.loginfo("Optical Flow Tracker (クリック指定版) 実行中... ESCで終了、左クリックで特徴点追加")
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("optical_flow_tracker_manual")
    topic = rospy.get_param("~topic", "/usb_cam/image_raw")
    path = rospy.get_param("~path", "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/image/calib_blackpoint.png")
    path = os.path.expanduser(path)
    tracker = OpticalFlowTracker(topic=topic, path=path)
    tracker.run()
