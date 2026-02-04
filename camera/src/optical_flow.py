#!/usr/bin/env python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

class OpticalFlowCalculator():
    def __init__(self):
        self.bridge = CvBridge()
        self.prev_gray = None
        self.tracking_point = None
        self.setup_ros()
        
    def setup_ros(self):
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.tracking_point = [x, y]
            rospy.loginfo(f"クリック追加: ({x}, {y})")
        
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.namedWindow("optical flow")
        cv2.setMouseCallback("optical flow", self.mouse_callback)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return
        
        flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                            0.5, 3, 15, 3, 5, 1.2, 0)
        fx = flow[..., 0]
        fy = flow[..., 1]
        
        mag, ang = cv2.cartToPolar(fx, fy)

        mag_threshold = 1.0
        mask = mag > mag_threshold

        if np.sum(mask) > 50:
            avg_fx = np.mean(fx[mask])
            avg_fy = np.mean(fy[mask])
            avg_speed = np.sqrt(avg_fx**2 + avg_fy**2)
            avg_angle = np.arctan2(avg_fy, avg_fx)
            
            # 表示用  
            print("平均速度ベクトル: (%.2f, %.2f)" % (avg_fx, avg_fy))
            print("平均速度の大きさ: %.2f" % avg_speed)
            print("平均方向 [rad]: %.2f" % avg_angle)
            print("平均方向 [deg]: %.2f" % (avg_angle * 180 / np.pi))
            print("----------------------------------")
        
        hsv = np.zeros_like(frame)
        hsv[..., 1] = 255
        hsv[..., 0] = ang * 180 / np.pi / 2
        hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        if self.tracking_point:
            self.tracking_point[0] += avg_fx
            self.tracking_point[1] += avg_fy
            cv2.circle(bgr, (int(self.tracking_point[0]), int(self.tracking_point[1])), 5, (0, 255, 0), -1)
        cv2.imshow('optical flow', bgr)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            print("Reset tracking point.")
            self.tracking_point = None
        
        self.prev_gray = gray

if __name__ == "__main__":
    rospy.init_node('optical_flow_node')
    try:
        Flow = OpticalFlowCalculator()
        rospy.spin()
    except:
        pass
