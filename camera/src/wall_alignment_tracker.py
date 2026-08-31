#!/usr/bin/env python3

import os
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool


class WallAlignmentTracker():
    def __init__(self, topic, compressed, ref_path, resize_width, resize_height,
                 max_iterations, epsilon, min_correlation):
        self.bridge = CvBridge()
        self.topic = topic
        self.compressed = compressed
        self.ref_path = ref_path
        self.resize_width = resize_width
        self.resize_height = resize_height
        self.max_iterations = max_iterations
        self.epsilon = epsilon
        self.min_correlation = min_correlation

        self.ref_gray_small = None
        self.warp_matrix = np.eye(2, 3, dtype=np.float32)
        self.last_frame = None

        self.load_reference(self.ref_path)
        self.setup_ros()

    def setup_ros(self):
        if self.compressed:
            self.sub = rospy.Subscriber(self.topic, CompressedImage, self.image_callback)
        else:
            self.sub = rospy.Subscriber(self.topic, Image, self.image_callback)
        self.pub_deviation = rospy.Publisher("/wall_alignment/deviation", Pose2D, queue_size=1)
        self.pub_valid = rospy.Publisher("/wall_alignment/valid", Bool, queue_size=1)
        self.pub_debug = rospy.Publisher("/processed_image/wall_alignment", Image, queue_size=1)

    def load_reference(self, path):
        if not os.path.exists(path):
            rospy.logwarn("Wall reference image not found at %s. Press 's' on the image window to capture one.", path)
            return
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            rospy.logwarn("Failed to load wall reference image at %s", path)
            return
        self.set_reference(img)
        rospy.loginfo("Loaded wall reference image from %s", path)

    def set_reference(self, bgr_image):
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        self.ref_gray_small = cv2.resize(gray, (self.resize_width, self.resize_height))
        self.warp_matrix = np.eye(2, 3, dtype=np.float32)

    def save_reference(self):
        if self.last_frame is None:
            rospy.logwarn("No frame available yet to capture as reference.")
            return
        cv2.imwrite(self.ref_path, self.last_frame)
        self.set_reference(self.last_frame)
        rospy.loginfo("Captured current frame as new wall reference: %s", self.ref_path)

    def align(self, gray_small):
        warp_mode = cv2.MOTION_EUCLIDEAN
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, self.max_iterations, self.epsilon)
        # ウォームスタート: 前フレームで収束した変換を初期値にすることで、
        # 小さいズレを前提に毎フレームの収束を速く安定させる。
        warp_matrix = self.warp_matrix.copy()
        try:
            correlation, warp_matrix = cv2.findTransformECC(
                self.ref_gray_small, gray_small, warp_matrix, warp_mode, criteria)
        except cv2.error:
            try:
                warp_matrix = np.eye(2, 3, dtype=np.float32)
                correlation, warp_matrix = cv2.findTransformECC(
                    self.ref_gray_small, gray_small, warp_matrix, warp_mode, criteria)
            except cv2.error:
                return None, 0.0
        return warp_matrix, correlation

    def extract_deviation(self, warp_matrix, full_shape):
        scale_x = full_shape[1] / self.resize_width
        scale_y = full_shape[0] / self.resize_height
        # warp_matrixはWARP_INVERSE_MAP前提の基準画像->現在フレームの変換。
        # tx, tyは現在フレームが基準からどれだけずれているかに対応する。
        dx = warp_matrix[0, 2] * scale_x
        dy = warp_matrix[1, 2] * scale_y
        dtheta = np.degrees(np.arctan2(warp_matrix[1, 0], warp_matrix[0, 0]))
        return dx, dy, dtheta

    def publish_deviation(self, dx, dy, dtheta):
        msg = Pose2D()
        msg.x = dx
        msg.y = dy
        msg.theta = dtheta
        self.pub_deviation.publish(msg)

    def draw_visualization(self, image, dx, dy, dtheta, correlation):
        h, w = image.shape[:2]
        center = (w // 2, h // 2)
        arrow_scale = 3
        end_point = (int(center[0] + dx * arrow_scale), int(center[1] + dy * arrow_scale))
        cv2.arrowedLine(image, center, end_point, (0, 0, 255), 3, tipLength=0.3)
        cv2.putText(image, "dx=%.1f dy=%.1f dtheta=%.2fdeg" % (dx, dy, dtheta),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(image, "corr=%.3f" % correlation,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def publish_debug_image(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.pub_debug.publish(msg)

    def decode_image(self, msg):
        if self.compressed:
            array = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def image_callback(self, msg):
        frame = self.decode_image(msg)
        self.last_frame = frame

        if self.ref_gray_small is None:
            cv2.imshow("Wall Alignment", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.save_reference()
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_small = cv2.resize(gray, (self.resize_width, self.resize_height))

        warp_matrix, correlation = self.align(gray_small)
        valid = warp_matrix is not None and correlation >= self.min_correlation

        debug_image = frame.copy()
        if valid:
            self.warp_matrix = warp_matrix
            dx, dy, dtheta = self.extract_deviation(warp_matrix, gray.shape)
            self.publish_deviation(dx, dy, dtheta)
            self.draw_visualization(debug_image, dx, dy, dtheta, correlation)
        else:
            rospy.logwarn_throttle(2.0, "Wall alignment lost (correlation=%.3f)" % correlation)
            self.warp_matrix = np.eye(2, 3, dtype=np.float32)

        self.pub_valid.publish(Bool(data=valid))
        self.publish_debug_image(debug_image)

        cv2.imshow("Wall Alignment", debug_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            self.save_reference()


if __name__ == "__main__":
    rospy.init_node("wall_alignment_tracker")
    compressed = rospy.get_param("~compressed", False)
    default_topic = "/usb_cam/image_raw/compressed" if compressed else "/usb_cam/image_raw"
    topic = rospy.get_param("~topic", default_topic)
    ref_path = rospy.get_param("~ref_path", "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/camera/src/image/wall_reference.png")
    ref_path = os.path.expanduser(ref_path)
    resize_width = rospy.get_param("~resize_width", 320)
    resize_height = rospy.get_param("~resize_height", 240)
    max_iterations = rospy.get_param("~max_iterations", 50)
    epsilon = rospy.get_param("~epsilon", 1e-4)
    min_correlation = rospy.get_param("~min_correlation", 0.6)

    tracker = WallAlignmentTracker(topic, compressed, ref_path, resize_width, resize_height,
                                    max_iterations, epsilon, min_correlation)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
