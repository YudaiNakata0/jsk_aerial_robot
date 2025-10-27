#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class EdgeDetection():
    def __init__(self):
        self.bridge = CvBridge()

        self.setup_ros()
        
    def setup_ros(self):
        self.sub = rospy.Subscriber("/processed_image/filtered_mask", Image, self.callback)
        self.pub = rospy.Publisher("/processed_image/edge", Image, queue_size=1)
        
    def callback(self, msg):
        try:
            # ROS Image → OpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
            return

        # --- 輪郭抽出処理 ---
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 輪郭描画
        output = cv_image.copy()
        cv2.drawContours(output, contours, -1, (0, 255, 0), 2)

        # # 結果表示
        # cv2.imshow("Contours", output)
        # cv2.waitKey(1)

        self.publish(output, self.pub)

    def publish(self, image, pub):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        pub.publish(image_msg)
        
        
if __name__ == '__main__':
    rospy.init_node("contour_extractor", anonymous=True)
    node = EdgeDetection()
    rospy.spin()
    cv2.destroyAllWindows()
