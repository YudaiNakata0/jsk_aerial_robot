#!/usr/bin/env python3
#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OpticalFlowTracker:
    def __init__(self, topic="/usb_cam/image_raw"):
        self.bridge = CvBridge()
        self.topic = topic

        # 特徴点検出パラメータ
        self.feature_params = dict(maxCorners=100,
                                   qualityLevel=0.3,
                                   minDistance=7,
                                   blockSize=7)

        # Lucas-Kanade法パラメータ
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS |
                                        cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # 状態管理
        self.prev_gray = None
        self.prev_points = None
        self.mask = None

        # ROS購読設定
        rospy.Subscriber(self.topic, Image, self.callback)

        # マウスクリックで特徴点設定
        # cv2.namedWindow("Optical Flow (ROS)")
        # cv2.setMouseCallback("Optical Flow (ROS)", self.mouse_callback)
        self.click_points = []

    def callback(self, msg):
        """画像トピックを受け取って処理"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 初回フレームまたは特徴点なしのとき
        if self.prev_gray is None or self.prev_points is None:
            # マウスクリックで特徴点設定
            cv2.namedWindow("Optical Flow (ROS)")
            cv2.setMouseCallback("Optical Flow (ROS)", self.mouse_callback)
            self.prev_gray = gray
            # self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.mask = np.zeros_like(frame)
            return

        # オプティカルフロー計算
        next_points, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None, **self.lk_params)

        # 良好な点のみ抽出
        if next_points is not None:
            good_new = next_points[status == 1]
            good_old = self.prev_points[status == 1]

            # 線と点の描画
            for new, old in zip(good_new, good_old):
                a, b = new.ravel()
                c, d = old.ravel()
                self.mask = cv2.line(self.mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 0, 255), -1)

            output = cv2.add(frame, self.mask)
            cv2.imshow("Optical Flow (ROS)", output)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rospy.signal_shutdown("User exited")
            elif key == ord('r'):  # 特徴点を再検出
                rospy.loginfo("特徴点を再検出します")
                self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
                self.mask = np.zeros_like(frame)
        else:
            # 再検出（追跡が途切れた場合）
            self.prev_points = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.mask = np.zeros_like(frame)

        # 状態更新
        self.prev_gray = gray
        if next_points is not None:
            self.prev_points = good_new.reshape(-1, 1, 2)

    def mouse_callback(self, event, x, y, flags, param):
        """マウスクリックで特徴点を追加"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_points.append([[x, y]])
            rospy.loginfo(f"クリック追加: ({x}, {y})")
            # numpy配列に変換（OpenCVが扱える形式に
            self.prev_points = np.array(self.click_points, dtype=np.float32)

    def run(self):
        """ROSスピン開始"""
        rospy.loginfo("Optical Flow Tracker (ROS) 実行中... ESCキーで終了")
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("optical_flow_tracker")
    tracker = OpticalFlowTracker(topic="/usb_cam/image_raw")
    rospy.spin()
    cv2.destroyAllWindows()
