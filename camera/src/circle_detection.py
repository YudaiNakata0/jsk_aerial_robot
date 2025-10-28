#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CircleDetector():
    def __init__(self, sub_topic):
        self.bridge = CvBridge()

        self.topic = sub_topic
        self.setup_ros()

    def setup_ros(self):
        self.sub_image = rospy.Subscriber(self.topic, Image, self.callback)
        
    def callback(self, msg):
        try:
            # ROS画像 → OpenCV画像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", e)
            return

        # グレースケール変換
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ノイズ除去
        gray = cv2.medianBlur(gray, 5)

        # 円検出（Hough変換）
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,              # 分解能の逆数
            minDist=30,          # 検出する円の中心同士の最小距離
            param1=100,          # Cannyの上限閾値
            param2=30,           # 円検出のしきい値（小さいほど検出しやすい）
            minRadius=5,         # 検出する円の最小半径
            maxRadius=100        # 検出する円の最大半径
        )

        # 検出結果の描画
        if circles is not None:
            circles = circles[0, :].astype(int)
            for (x, y, r) in circles:
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)

            rospy.loginfo(f"Detected {len(circles)} circles")

        # 表示（必要ならコメントアウト解除）
        cv2.imshow("Detected Circles", cv_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("circle_detection_node")
    topic = rospy.get_param("~topic", "/processed_image/filtered_mask")
    detector = CircleDetector(topic)
    rospy.spin()
    cv2.destroyAllWindows()
