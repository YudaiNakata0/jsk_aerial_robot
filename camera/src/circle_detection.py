#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3

class CircleDetector():
    def __init__(self, sub_topic):
        self.bridge = CvBridge()

        self.topic = sub_topic
        self.setup_ros()
        path = os.path.expanduser("~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/image/calib_blackpoint.png")
        self.is_mask_exist = False
        self.load_calib_image(path)

    def setup_ros(self):
        self.sub_image = rospy.Subscriber(self.topic, Image, self.callback)
        self.pub_circle = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)
        self.pub_circle_image = rospy.Publisher("/processed_image/circle", Image, queue_size=1)

    # サンプル画像（白黒）の読み込み
    def load_calib_image(self, path):
        self.mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if self.mask is None:
            print("failed load mask image")
        else:
            self.is_mask_exist = True
        
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
        # cv2.imshow("Grayscale", gray)

        # mask
        if self.is_mask_exist:
            gray_masked = cv2.bitwise_and(gray, gray, mask=cv2.bitwise_not(self.mask))
            # cv2.imshow("Masked Grayscale", gray_masked)
        else:
            pass

        # 円検出（Hough変換）
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,              # 分解能の逆数
            minDist=30,          # 検出する円の中心同士の最小距離
            param1=100,          # Cannyの上限閾値
            param2=40,           # 円検出のしきい値（小さいほど検出しやすい）
            minRadius=30,        # 検出する円の最小半径
            maxRadius=50        # 検出する円の最大半径
        )

        # 検出結果の描画
        if circles is not None:
            circles = circles[0, :].astype(int)
            print(circles)
            for (x, y, r) in circles:
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(cv_image, (x, y), 2, (0, 0, 255), 3)

            largest_circle = max(circles, key=lambda x: x[2])
            msg = Vector3()
            msg.x = largest_circle[0]
            msg.y = largest_circle[1]
            msg.z = largest_circle[2]
            self.pub_circle.publish(msg)

            rospy.loginfo(f"Detected {len(circles)} circles")

        # publish
        circle_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.pub_circle_image.publish(circle_image_msg)

        # 表示
        # cv2.imshow("Detected Circles", cv_image)
        # cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("circle_detection_node")
    topic = rospy.get_param("~topic", "/processed_image/filtered_mask")
    detector = CircleDetector(topic)
    rospy.spin()
    cv2.destroyAllWindows()
