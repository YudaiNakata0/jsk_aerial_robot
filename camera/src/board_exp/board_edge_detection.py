#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Bool


class BoardEdgeDetector():
    def __init__(self, topic, compressed, black_thresh, min_area, approx_epsilon_ratio, blur_ksize,
                 expected_length, length_tolerance, max_angle_from_horizontal_deg,
                 search_top_ratio, auto_threshold, min_black_thresh, max_black_thresh):
        self.bridge = CvBridge()
        self.topic = topic
        self.compressed = compressed
        self.black_thresh = black_thresh
        self.min_area = min_area
        self.approx_epsilon_ratio = approx_epsilon_ratio
        self.blur_ksize = blur_ksize
        self.expected_length = expected_length
        self.length_tolerance = length_tolerance
        self.max_angle_from_horizontal_deg = max_angle_from_horizontal_deg
        self.search_top_ratio = search_top_ratio
        self.auto_threshold = auto_threshold
        self.min_black_thresh = min_black_thresh
        self.max_black_thresh = max_black_thresh

        self.setup_ros()

    def setup_ros(self):
        if self.compressed:
            self.sub = rospy.Subscriber(self.topic, CompressedImage, self.image_callback)
        else:
            self.sub = rospy.Subscriber(self.topic, Image, self.image_callback)
        self.pub_mask = rospy.Publisher("/processed_image/board_mask", Image, queue_size=1)
        self.pub_edge = rospy.Publisher("/processed_image/board_edge", Image, queue_size=1)
        self.pub_bottom_edge = rospy.Publisher("/target/board_bottom_edge", PolygonStamped, queue_size=1)
        self.pub_valid = rospy.Publisher("/target/board_bottom_edge_valid", Bool, queue_size=1)

    def decode_image(self, msg):
        if self.compressed:
            array = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def compute_threshold(self, blurred):
        if not self.auto_threshold:
            return float(self.black_thresh)
        # Otsuの二値化は現在のフレームの明暗2クラス（黒いエリア/背景）の谷を
        # 自動で見つけるため、暗い現場で全体の明るさが下がってもその都度
        # 追従できる。ただし極端な誤検出を避けるため、既知の妥当な範囲
        # [min_black_thresh, max_black_thresh] にクリップする。
        otsu_thresh, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        return float(np.clip(otsu_thresh, self.min_black_thresh, self.max_black_thresh))

    def extract_board_mask(self, gray):
        # 壁の背景やロボット自身の映り込みは中間〜明るいグレー、
        # 黒いエリア（壁の対象部分）だけが暗いグレースケール値になる前提で二値化する。
        blurred = cv2.medianBlur(gray, self.blur_ksize)
        thresh_val = self.compute_threshold(blurred)
        _, mask = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask, thresh_val

    def find_board_contour(self, mask):
        # 円形の穴の縁は無視し、黒いエリア全体の外周だけを取り出す。
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None
        perimeter = cv2.arcLength(largest, True)
        epsilon = self.approx_epsilon_ratio * perimeter
        return cv2.approxPolyDP(largest, epsilon, True)

    @staticmethod
    def normalize_angle(angle_deg):
        # 直線には向きがないので、水平からのズレを(-90, 90]度に正規化する。
        if angle_deg > 90:
            angle_deg -= 180
        elif angle_deg < -90:
            angle_deg += 180
        return angle_deg

    def find_bottom_edge(self, polygon):
        # 黒い四角の各辺のうち、「ほぼ水平」かつ「長さがだいたい期待通り」という
        # 条件を満たすものだけを候補にし、その中で画像内で最も下側（yが最大）にある
        # 辺を四角の下の縁とみなす。ロボットの映り込みで上辺に切れ込みができても、
        # 短い辺として長さ条件で弾かれるため下の縁の検出には影響しない。
        points = polygon.reshape(-1, 2)
        n = len(points)
        candidates = []
        for i in range(n):
            p1 = points[i]
            p2 = points[(i + 1) % n]
            dx = float(p2[0] - p1[0])
            dy = float(p2[1] - p1[1])
            length = float(np.hypot(dx, dy))
            if length < 1e-3:
                continue
            angle = self.normalize_angle(np.degrees(np.arctan2(dy, dx)))
            if abs(angle) > self.max_angle_from_horizontal_deg:
                continue
            if abs(length - self.expected_length) > self.length_tolerance:
                continue
            mean_y = (p1[1] + p2[1]) / 2.0
            candidates.append((mean_y, p1, p2))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0])
        _, p1, p2 = candidates[-1]
        return p1, p2

    def publish_bottom_edge(self, p1, p2, header):
        msg = PolygonStamped()
        msg.header = header
        msg.polygon.points.append(Point32(x=float(p1[0]), y=float(p1[1]), z=0.0))
        msg.polygon.points.append(Point32(x=float(p2[0]), y=float(p2[1]), z=0.0))
        self.pub_bottom_edge.publish(msg)

    def publish_mask(self, mask, header):
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_msg.header = header
        self.pub_mask.publish(mask_msg)

    def draw_visualization(self, image, contour, bottom_edge, roi_y0, thresh_val):
        h, w = image.shape[:2]
        cv2.line(image, (0, roi_y0), (w - 1, roi_y0), (255, 255, 0), 1)
        if contour is not None:
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 1)
        if bottom_edge is not None:
            p1, p2 = bottom_edge
            cv2.line(image, tuple(p1.astype(int)), tuple(p2.astype(int)), (0, 0, 255), 3)
            cv2.circle(image, tuple(p1.astype(int)), 5, (0, 255, 255), -1)
            cv2.circle(image, tuple(p2.astype(int)), 5, (0, 255, 255), -1)
        cv2.putText(image, "black_thresh=%.1f" % thresh_val, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def publish_debug_image(self, image, header):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self.pub_edge.publish(msg)

    def image_callback(self, msg):
        frame = self.decode_image(msg)
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 下の縁は画像の下半分にしか現れない前提で、探索範囲をそこに限定する。
        # 上半分に映り込むロボットや背景の暗い部分を誤って拾うのを防げるほか、
        # 処理範囲が減るぶん誤検出の機会そのものも減らせる。
        roi_y0 = int(h * self.search_top_ratio)
        gray_roi = gray[roi_y0:, :]

        mask_roi, thresh_val = self.extract_board_mask(gray_roi)
        contour_roi = self.find_board_contour(mask_roi)
        if contour_roi is not None:
            contour = contour_roi.copy()
            contour[:, :, 1] += roi_y0
        else:
            contour = None
        bottom_edge = self.find_bottom_edge(contour) if contour is not None else None
        valid = bottom_edge is not None

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[roi_y0:, :] = mask_roi

        self.pub_valid.publish(Bool(data=valid))
        self.publish_mask(mask, msg.header)

        debug_image = frame.copy()
        if valid:
            self.publish_bottom_edge(bottom_edge[0], bottom_edge[1], msg.header)
        else:
            rospy.logwarn_throttle(2.0, "Board bottom edge not found (thresh=%.1f, min_area=%d, "
                                    "expected_length=%.1f)" %
                                    (thresh_val, self.min_area, self.expected_length))
        self.draw_visualization(debug_image, contour, bottom_edge, roi_y0, thresh_val)

        self.publish_debug_image(debug_image, msg.header)


if __name__ == "__main__":
    rospy.init_node("board_edge_detection")
    compressed = rospy.get_param("~compressed", False)
    default_topic = "/usb_cam/image_raw/compressed" if compressed else "/usb_cam/image_raw"
    topic = rospy.get_param("~topic", default_topic)
    black_thresh = rospy.get_param("~black_thresh", 80)
    min_area = rospy.get_param("~min_area", 2000)
    approx_epsilon_ratio = rospy.get_param("~approx_epsilon_ratio", 0.01)
    blur_ksize = rospy.get_param("~blur_ksize", 5)
    expected_length = rospy.get_param("~expected_length", 300.0)
    length_tolerance = rospy.get_param("~length_tolerance", 150.0)
    max_angle_from_horizontal_deg = rospy.get_param("~max_angle_from_horizontal_deg", 15.0)
    search_top_ratio = rospy.get_param("~search_top_ratio", 0.5)
    auto_threshold = rospy.get_param("~auto_threshold", True)
    min_black_thresh = rospy.get_param("~min_black_thresh", 20)
    max_black_thresh = rospy.get_param("~max_black_thresh", 100)

    detector = BoardEdgeDetector(topic, compressed, black_thresh, min_area, approx_epsilon_ratio, blur_ksize,
                                  expected_length, length_tolerance, max_angle_from_horizontal_deg,
                                  search_top_ratio, auto_threshold, min_black_thresh, max_black_thresh)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
