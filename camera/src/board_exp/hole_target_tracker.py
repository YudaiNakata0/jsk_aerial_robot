#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3, Pose2D
from std_msgs.msg import Bool


class HoleTargetTracker():
    """
    1. サンプル画像を1枚キャプチャ ('s')
    2. そのサンプル画像上で穴の中心をクリック指定
    3. 穴そのものではなく、穴より下側の壁面（ロボットの映り込みが無く、
       壁のパターンも含められる範囲）を広めにテンプレートとして切り出し、
       以後のフレームでその移動を追跡する。
    4. トラッキング中に基準位置設定キー ('c') を押してからライブ映像上を
       クリックすると、そのクリック位置が偏差(/target/hole_deviation)算出の
       基準位置として更新される。穴指定時と異なり画面を止める必要はない。
       この基準位置はリセットキー ('r') を押しても保持され、穴の再指定の
       影響を受けない。
    """

    STATE_CAPTURE = "capture"
    STATE_SELECT = "select"
    STATE_TRACKING = "tracking"

    def __init__(self, topic, compressed, patch_width, patch_height, patch_top_margin,
                 match_thresh, search_margin, use_coarse_reacquire, coarse_scale,
                 lost_frames_before_full_search, capture_key, reset_key, set_reference_key):
        self.bridge = CvBridge()
        self.topic = topic
        self.compressed = compressed
        self.patch_width = patch_width
        self.patch_height = patch_height
        self.patch_top_margin = patch_top_margin
        self.match_thresh = match_thresh
        self.search_margin = search_margin
        self.use_coarse_reacquire = use_coarse_reacquire
        self.coarse_scale = coarse_scale
        self.lost_frames_before_full_search = lost_frames_before_full_search
        self.capture_key = ord(capture_key)
        self.reset_key = ord(reset_key)
        self.set_reference_key = ord(set_reference_key)

        self.window_name = "Hole Target Tracker"
        self.window_ready = False
        self.reset_state()
        self.setup_ros()

    def reset_state(self, preserve_reference=False):
        self.state = self.STATE_CAPTURE
        self.captured_frame = None
        self.click_point = None
        self.template_gray = None
        self.hole_offset = None
        if not preserve_reference:
            self.reference_point = None
        self.last_top_left = None
        self.lost_count = 0
        self.awaiting_reference_click = False

    def setup_ros(self):
        if self.compressed:
            self.sub = rospy.Subscriber(self.topic, CompressedImage, self.image_callback)
        else:
            self.sub = rospy.Subscriber(self.topic, Image, self.image_callback)
        self.pub_position = rospy.Publisher("/target/2D_position", Vector3, queue_size=1)
        self.pub_deviation = rospy.Publisher("/target/hole_deviation", Pose2D, queue_size=1)
        self.pub_valid = rospy.Publisher("/target/hole_tracking_valid", Bool, queue_size=1)
        self.pub_debug = rospy.Publisher("/processed_image/hole_tracking", Image, queue_size=1)

    def decode_image(self, msg):
        if self.compressed:
            array = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.state == self.STATE_SELECT:
            self.click_point = (x, y)
        elif self.state == self.STATE_TRACKING and self.awaiting_reference_click:
            # ライブ映像上のクリック位置をそのまま新しい基準位置とする。
            # 穴指定時と違い、テンプレートマッチングは行わないため画面を
            # 止める必要はない。
            self.reference_point = (x, y)
            self.awaiting_reference_click = False
            rospy.loginfo("Reference position updated to (%d, %d)." % (x, y))

    # ---------------- capture / select ----------------

    def handle_capture(self, frame):
        display = frame.copy()
        cv2.putText(display, "Press 's' to capture sample image", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(self.window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == self.capture_key:
            self.captured_frame = frame.copy()
            self.state = self.STATE_SELECT
            rospy.loginfo("Sample image captured. Click the center of the hole.")

    def handle_select(self, _frame):
        display = self.captured_frame.copy()
        cv2.putText(display, "Click the center of the hole", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)

        if self.click_point is None:
            return

        if self.setup_template(self.captured_frame, self.click_point):
            self.state = self.STATE_TRACKING
            rospy.loginfo("Template captured below the hole. Tracking started.")
        else:
            rospy.logwarn("Patch region goes out of image bounds, click again.")
            self.click_point = None

    def setup_template(self, frame, click_point):
        # 穴自体やその周囲上部はロボットの映り込みで使いにくいため避け、
        # 穴から`patch_top_margin`だけ下側を、壁のパターンも入るよう
        # 横に広め(patch_width)のテンプレートとして切り出す。
        h, w = frame.shape[:2]
        cx, cy = click_point
        px0 = int(cx - self.patch_width / 2)
        py0 = int(cy + self.patch_top_margin)
        px0 = max(0, min(px0, w - self.patch_width))
        py0 = max(0, min(py0, h - self.patch_height))
        px1 = px0 + self.patch_width
        py1 = py0 + self.patch_height
        if px1 > w or py1 > h:
            return False

        template_bgr = frame[py0:py1, px0:px1]
        self.template_gray = cv2.cvtColor(template_bgr, cv2.COLOR_BGR2GRAY)
        # 穴の位置をテンプレート左上からの相対オフセットとして保持し、
        # 追跡結果（テンプレートの位置）から穴の位置を逆算できるようにする。
        self.hole_offset = (cx - px0, cy - py0)
        # 基準位置は初回のみ穴の位置から設定する。'c'キーによる更新や、
        # リセット後に保持された値がある場合はここで上書きしない。
        if self.reference_point is None:
            self.reference_point = (cx, cy)
        self.last_top_left = (px0, py0)
        self.lost_count = 0
        return True

    # ---------------- tracking ----------------

    def match(self, gray, top_left, bottom_right):
        x0, y0 = top_left
        x1, y1 = bottom_right
        window = gray[y0:y1, x0:x1]
        if window.shape[0] < self.template_gray.shape[0] or window.shape[1] < self.template_gray.shape[1]:
            return None, 0.0
        result = cv2.matchTemplate(window, self.template_gray, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        return (max_loc[0] + x0, max_loc[1] + y0), max_val

    def windowed_search(self, gray):
        h, w = gray.shape[:2]
        th, tw = self.template_gray.shape[:2]
        lx, ly = self.last_top_left
        x0 = max(0, lx - self.search_margin)
        y0 = max(0, ly - self.search_margin)
        x1 = min(w, lx + tw + self.search_margin)
        y1 = min(h, ly + th + self.search_margin)
        return self.match(gray, (x0, y0), (x1, y1))

    def full_frame_search(self, gray):
        h, w = gray.shape[:2]
        th, tw = self.template_gray.shape[:2]
        if not self.use_coarse_reacquire:
            return self.match(gray, (0, 0), (w, h))

        # テンプレートが大きいと全画面探索は重いので、まず縮小画像で
        # おおよその位置を求め、その周辺だけ原寸で再探索して精度を出す。
        scale = self.coarse_scale
        small_gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        small_template = cv2.resize(self.template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        result = cv2.matchTemplate(small_gray, small_template, cv2.TM_CCOEFF_NORMED)
        _, _, _, max_loc = cv2.minMaxLoc(result)
        coarse_top_left = (int(max_loc[0] / scale), int(max_loc[1] / scale))

        x0 = max(0, coarse_top_left[0] - self.search_margin)
        y0 = max(0, coarse_top_left[1] - self.search_margin)
        x1 = min(w, coarse_top_left[0] + tw + self.search_margin)
        y1 = min(h, coarse_top_left[1] + th + self.search_margin)
        return self.match(gray, (x0, y0), (x1, y1))

    def handle_tracking(self, frame, gray, header):
        if self.last_top_left is not None and self.lost_count < self.lost_frames_before_full_search:
            top_left, score = self.windowed_search(gray)
        else:
            top_left, score = self.full_frame_search(gray)

        valid = top_left is not None and score >= self.match_thresh

        if valid:
            self.last_top_left = top_left
            self.lost_count = 0
            hole_x = top_left[0] + self.hole_offset[0]
            hole_y = top_left[1] + self.hole_offset[1]
            dx = hole_x - self.reference_point[0]
            dy = hole_y - self.reference_point[1]
            self.publish_position(hole_x, hole_y, score)
            self.publish_deviation(dx, dy)
        else:
            self.lost_count += 1
            rospy.logwarn_throttle(2.0, "Hole tracking lost (score=%.3f, lost_count=%d)" %
                                    (score, self.lost_count))

        self.pub_valid.publish(Bool(data=valid))
        debug_image = frame.copy()
        self.draw_visualization(debug_image, top_left, score, valid)
        self.publish_debug_image(debug_image, header)

        key = cv2.waitKey(1) & 0xFF
        if key == self.reset_key:
            rospy.loginfo("Reset requested. Capture a new sample image. Reference position is kept.")
            self.reset_state(preserve_reference=True)
        elif key == self.set_reference_key:
            self.awaiting_reference_click = True
            rospy.loginfo("Click on the live image to set the new reference position.")

    # ---------------- publish / draw ----------------

    def publish_position(self, x, y, score):
        msg = Vector3(x=float(x), y=float(y), z=float(score))
        self.pub_position.publish(msg)

    def publish_deviation(self, dx, dy):
        msg = Pose2D(x=float(dx), y=float(dy), theta=0.0)
        self.pub_deviation.publish(msg)

    def draw_visualization(self, image, top_left, score, valid):
        th, tw = self.template_gray.shape[:2]
        if top_left is not None:
            color = (0, 255, 0) if valid else (0, 0, 255)
            cv2.rectangle(image, top_left, (top_left[0] + tw, top_left[1] + th), color, 2)
            if valid:
                hole_x = top_left[0] + self.hole_offset[0]
                hole_y = top_left[1] + self.hole_offset[1]
                cv2.circle(image, (int(hole_x), int(hole_y)), 6, (0, 0, 255), -1)
        if self.reference_point is not None:
            rx, ry = self.reference_point
            cv2.drawMarker(image, (int(rx), int(ry)), (255, 0, 0),
                            markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
        cv2.putText(image, "score=%.3f" % score, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if self.awaiting_reference_click:
            cv2.putText(image, "Click to set reference position", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        else:
            cv2.putText(image, "Press '%c' to set reference position" % self.set_reference_key, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.imshow(self.window_name, image)

    def publish_debug_image(self, image, header):
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header = header
        self.pub_debug.publish(msg)

    # ---------------- main callback ----------------

    def image_callback(self, msg):
        frame = self.decode_image(msg)

        if not self.window_ready:
            # imshowはウィンドウを自動生成するが、setMouseCallbackは
            # ウィンドウが存在してから呼ぶ必要があるため、namedWindowで
            # 明示的に先に作ってからクリックコールバックを登録する。
            cv2.namedWindow(self.window_name)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.window_ready = True

        if self.state == self.STATE_CAPTURE:
            self.handle_capture(frame)
            return
        if self.state == self.STATE_SELECT:
            self.handle_select(frame)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.handle_tracking(frame, gray, msg.header)


if __name__ == "__main__":
    rospy.init_node("hole_target_tracker")
    compressed = rospy.get_param("~compressed", False)
    default_topic = "/usb_cam/image_raw/compressed" if compressed else "/usb_cam/image_raw"
    topic = rospy.get_param("~topic", default_topic)
    patch_width = rospy.get_param("~patch_width", 480)
    patch_height = rospy.get_param("~patch_height", 200)
    patch_top_margin = rospy.get_param("~patch_top_margin", 40)
    match_thresh = rospy.get_param("~match_thresh", 0.7)
    search_margin = rospy.get_param("~search_margin", 60)
    use_coarse_reacquire = rospy.get_param("~use_coarse_reacquire", True)
    coarse_scale = rospy.get_param("~coarse_scale", 0.5)
    lost_frames_before_full_search = rospy.get_param("~lost_frames_before_full_search", 5)
    capture_key = rospy.get_param("~capture_key", "s")
    reset_key = rospy.get_param("~reset_key", "r")
    set_reference_key = rospy.get_param("~set_reference_key", "c")

    tracker = HoleTargetTracker(topic, compressed, patch_width, patch_height, patch_top_margin,
                                 match_thresh, search_margin, use_coarse_reacquire, coarse_scale,
                                 lost_frames_before_full_search, capture_key, reset_key,
                                 set_reference_key)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
