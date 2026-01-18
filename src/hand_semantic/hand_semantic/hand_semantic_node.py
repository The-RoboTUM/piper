#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import mediapipe as mp


class HandSemanticNode(Node):
    """
    Semantic hand detector using MediaPipe Hands with:
      - ROI gating (down-middle of frame)
      - min % of hand inside ROI (landmark-based)
      - depth gating (wrist depth in meters)

    Publishes:
      - /hand/semantic_detected (Bool)
      - /hand/annotated_image (Image BGR8)
    """

    def __init__(self) -> None:
        super().__init__("hand_semantic_node")

        # -----------------------
        # ROS / MediaPipe params
        # -----------------------
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_rect_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("publish_annotated", True)

        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)
        self.declare_parameter("max_num_hands", 1)

        # -----------------------
        # ROI params (normalized)
        # ROI is forced to be "down middle" by default (lock_center_x=True)
        # -----------------------
        self.declare_parameter("roi_enabled", True)
        self.declare_parameter("roi_lock_center_x", True)  # force x center to 0.5
        self.declare_parameter("roi_center_x", 0.6)        # normalized 0..1
        self.declare_parameter("roi_center_y", 0.8)       # lower-middle
        self.declare_parameter("roi_width", 0.40)          # narrow vertical band
        self.declare_parameter("roi_height", 0.35)         # tall area
        # How strict is "80% hand inside ROI"
        self.declare_parameter("roi_min_inside_ratio", 0.70)  # 80%
        # method: landmark_ratio (recommended) or bbox_overlap
        self.declare_parameter("roi_policy", "landmark_ratio")

        # -----------------------
        # Depth params
        # -----------------------
        self.declare_parameter("depth_enabled", True)
        self.declare_parameter("min_depth_m", 0.15)  # 10 cm
        self.declare_parameter("max_depth_m", 0.30)  # 30 cm
        self.declare_parameter("depth_sync_tolerance_s", 0.10)  # accept depth within 100ms
        self.declare_parameter("depth_sample_window", 7)        # odd number recommended
        # depth encoding auto-handling:
        # - 16UC1 assumed mm
        # - 32FC1 assumed meters
        self.declare_parameter("depth_invalid_value_m", 0.0)    # if depth==0 treat invalid

        # Read params
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)

        self.min_det_conf = float(self.get_parameter("min_detection_confidence").value)
        self.min_trk_conf = float(self.get_parameter("min_tracking_confidence").value)
        self.max_num_hands = int(self.get_parameter("max_num_hands").value)

        self.roi_enabled = bool(self.get_parameter("roi_enabled").value)
        self.roi_lock_center_x = bool(self.get_parameter("roi_lock_center_x").value)
        self.roi_cx = float(self.get_parameter("roi_center_x").value)
        self.roi_cy = float(self.get_parameter("roi_center_y").value)
        self.roi_w = float(self.get_parameter("roi_width").value)
        self.roi_h = float(self.get_parameter("roi_height").value)
        self.roi_min_inside_ratio = float(self.get_parameter("roi_min_inside_ratio").value)
        self.roi_policy = str(self.get_parameter("roi_policy").value)

        self.depth_enabled = bool(self.get_parameter("depth_enabled").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.depth_sync_tol = float(self.get_parameter("depth_sync_tolerance_s").value)
        self.depth_window = int(self.get_parameter("depth_sample_window").value)
        self.depth_invalid_value_m = float(self.get_parameter("depth_invalid_value_m").value)

        # Force ROI down-middle (center_x fixed)
        if self.roi_lock_center_x:
            self.roi_cx = 0.5

        # Clamp sanity
        self.roi_cx = float(np.clip(self.roi_cx, 0.0, 1.0))
        self.roi_cy = float(np.clip(self.roi_cy, 0.0, 1.0))
        self.roi_w = float(np.clip(self.roi_w, 0.01, 1.0))
        self.roi_h = float(np.clip(self.roi_h, 0.01, 1.0))
        self.roi_min_inside_ratio = float(np.clip(self.roi_min_inside_ratio, 0.0, 1.0))
        if self.depth_window < 1:
            self.depth_window = 1
        if self.depth_window % 2 == 0:
            self.depth_window += 1  # make it odd

        # ROS I/O
        self.bridge = CvBridge()
        self.pub_detected = self.create_publisher(Bool, "/hand/semantic_detected", 10)
        self.pub_annot = self.create_publisher(Image, "/hand/annotated_image", 10)

        self.sub_rgb = self.create_subscription(
            Image, self.rgb_topic, self.on_rgb, qos_profile_sensor_data
        )

        # Depth cache (latest)
        self._depth_msg: Optional[Image] = None
        self._depth_stamp_s: Optional[float] = None
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self.on_depth, qos_profile_sensor_data
        )

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_num_hands,
            model_complexity=1,
            min_detection_confidence=self.min_det_conf,
            min_tracking_confidence=self.min_trk_conf,
        )

        self.get_logger().info(
            f"HandSemanticNode up.\n"
            f" RGB:   {self.rgb_topic}\n"
            f" Depth: {self.depth_topic}\n"
            f" ROI: enabled={self.roi_enabled}, center=({self.roi_cx:.2f},{self.roi_cy:.2f}), "
            f" size=({self.roi_w:.2f},{self.roi_h:.2f}), min_inside={self.roi_min_inside_ratio:.2f}, "
            f" policy={self.roi_policy}, lock_center_x={self.roi_lock_center_x}\n"
            f" Depth gating: enabled={self.depth_enabled}, range=[{self.min_depth_m:.2f},{self.max_depth_m:.2f}] m"
        )

    # ---------- Depth handling ----------
    def on_depth(self, msg: Image) -> None:
        # cache last depth message + stamp in seconds
        self._depth_msg = msg
        self._depth_stamp_s = self._ros_time_to_float_s(msg.header.stamp)

    @staticmethod
    def _ros_time_to_float_s(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _get_depth_cv(self) -> Optional[np.ndarray]:
        """
        Returns depth image as numpy array (raw), or None if unavailable.
        Supports:
          - 16UC1 (depth in mm)
          - 32FC1 (depth in meters)
        """
        if self._depth_msg is None:
            return None

        try:
            # passthrough keeps original encoding
            depth = self.bridge.imgmsg_to_cv2(self._depth_msg, desired_encoding="passthrough")
            return np.array(depth)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge depth conversion failed: {e}")
            return None

    def _depth_at_pixel_m(self, depth_img: np.ndarray, x: int, y: int, encoding: str) -> Optional[float]:
        """
        Robust depth sampling at (x,y):
          - median in NxN window
          - ignore invalid values (0, NaN, inf)
        Converts:
          - 16UC1: mm -> meters
          - 32FC1: meters
        """
        h, w = depth_img.shape[:2]
        if x < 0 or x >= w or y < 0 or y >= h:
            return None

        half = self.depth_window // 2
        x1 = max(0, x - half)
        x2 = min(w, x + half + 1)
        y1 = max(0, y - half)
        y2 = min(h, y + half + 1)

        patch = depth_img[y1:y2, x1:x2].astype(np.float32)

        # Convert units depending on encoding
        # Common RealSense:
        # - 16UC1 in millimeters
        # - 32FC1 in meters
        if encoding == "16UC1":
            patch_m = patch * 0.001
        else:
            patch_m = patch

        # Filter invalid
        good = patch_m[np.isfinite(patch_m)]
        good = good[good > self.depth_invalid_value_m + 1e-9]  # treat 0 as invalid by default

        if good.size == 0:
            return None

        return float(np.median(good))

    # ---------- ROI + hand logic ----------
    def _roi_rect_px(self, img_w: int, img_h: int) -> Tuple[int, int, int, int]:
        cx = self.roi_cx * img_w
        cy = self.roi_cy * img_h
        rw = self.roi_w * img_w
        rh = self.roi_h * img_h

        x1 = int(max(0, cx - rw / 2))
        y1 = int(max(0, cy - rh / 2))
        x2 = int(min(img_w - 1, cx + rw / 2))
        y2 = int(min(img_h - 1, cy + rh / 2))
        return x1, y1, x2, y2

    @staticmethod
    def _point_in_rect(x: int, y: int, rect: Tuple[int, int, int, int]) -> bool:
        x1, y1, x2, y2 = rect
        return (x1 <= x <= x2) and (y1 <= y <= y2)

    def _landmark_inside_ratio(
        self,
        hand_landmarks,
        roi_rect: Tuple[int, int, int, int],
        img_w: int,
        img_h: int
    ) -> float:
        """
        Ratio of landmarks that fall inside ROI.
        MediaPipe Hands has 21 landmarks.
        """
        inside = 0
        total = 0
        for lm in hand_landmarks.landmark:
            x = int(lm.x * img_w)
            y = int(lm.y * img_h)
            total += 1
            if self._point_in_rect(x, y, roi_rect):
                inside += 1
        return float(inside) / float(total) if total > 0 else 0.0

    def _bbox_overlap_ratio(
        self,
        hand_landmarks,
        roi_rect: Tuple[int, int, int, int],
        img_w: int,
        img_h: int
    ) -> float:
        """
        Approx “how much of hand is inside ROI” using bbox overlap:
        overlap_area(hand_bbox, roi) / area(hand_bbox)
        """
        xs = []
        ys = []
        for lm in hand_landmarks.landmark:
            xs.append(int(lm.x * img_w))
            ys.append(int(lm.y * img_h))

        if not xs or not ys:
            return 0.0

        hx1, hx2 = min(xs), max(xs)
        hy1, hy2 = min(ys), max(ys)
        hand_area = max(1, (hx2 - hx1 + 1) * (hy2 - hy1 + 1))

        rx1, ry1, rx2, ry2 = roi_rect
        ox1 = max(hx1, rx1)
        oy1 = max(hy1, ry1)
        ox2 = min(hx2, rx2)
        oy2 = min(hy2, ry2)

        if ox2 < ox1 or oy2 < oy1:
            return 0.0

        overlap_area = (ox2 - ox1 + 1) * (oy2 - oy1 + 1)
        return float(overlap_area) / float(hand_area)

    def _depth_ok_for_hand(
        self,
        hand_landmarks,
        img_w: int,
        img_h: int,
        rgb_stamp_s: float
    ) -> Tuple[bool, Optional[float], str]:
        """
        Returns (ok, depth_m, reason).
        Depth sampled at WRIST landmark (index 0).
        """
        if not self.depth_enabled:
            return True, None, "depth_disabled"

        if self._depth_msg is None or self._depth_stamp_s is None:
            return False, None, "no_depth_msg"

        # check time closeness (simple cache sync)
        if abs(self._depth_stamp_s - rgb_stamp_s) > self.depth_sync_tol:
            return False, None, "depth_out_of_sync"

        depth_img = self._get_depth_cv()
        if depth_img is None:
            return False, None, "depth_cv_none"

        # wrist landmark index = 0
        wrist = hand_landmarks.landmark[0]
        px = int(wrist.x * img_w)
        py = int(wrist.y * img_h)

        encoding = str(self._depth_msg.encoding)  # e.g., "16UC1" or "32FC1"
        depth_m = self._depth_at_pixel_m(depth_img, px, py, encoding)

        if depth_m is None:
            return False, None, "depth_invalid"

        if self.min_depth_m <= depth_m <= self.max_depth_m:
            return True, depth_m, "depth_ok"

        return False, depth_m, "depth_out_of_range"

    # ---------- Main callback ----------
    def on_rgb(self, msg: Image) -> None:
        rgb_stamp_s = self._ros_time_to_float_s(msg.header.stamp)

        # Convert ROS Image -> OpenCV BGR
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge RGB conversion failed: {e}")
            return

        img_h, img_w = bgr.shape[:2]
        roi_rect = self._roi_rect_px(img_w, img_h) if self.roi_enabled else None

        # MediaPipe expects RGB
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        hand_found = bool(results.multi_hand_landmarks)

        # Evaluate "meaningful" with ROI + ratio + depth
        meaningful = False
        best_ratio = 0.0
        best_depth_m: Optional[float] = None
        best_reason = "no_hand"
        best_hand_landmarks = None

        if hand_found:
            for hl in results.multi_hand_landmarks:
                # ROI ratio
                if self.roi_enabled and roi_rect is not None:
                    if self.roi_policy == "bbox_overlap":
                        ratio = self._bbox_overlap_ratio(hl, roi_rect, img_w, img_h)
                    else:
                        ratio = self._landmark_inside_ratio(hl, roi_rect, img_w, img_h)
                else:
                    ratio = 1.0  # if ROI disabled, treat as fully inside

                # Depth gate
                depth_ok, depth_m, depth_reason = self._depth_ok_for_hand(hl, img_w, img_h, rgb_stamp_s)

                # Must satisfy both: ratio + depth
                ok = (ratio >= self.roi_min_inside_ratio) and depth_ok

                # Track "best" hand for debugging overlay
                if ratio > best_ratio:
                    best_ratio = ratio
                    best_depth_m = depth_m
                    best_reason = depth_reason if ratio >= self.roi_min_inside_ratio else "roi_ratio_too_low"
                    best_hand_landmarks = hl

                if ok:
                    meaningful = True
                    best_ratio = ratio
                    best_depth_m = depth_m
                    best_reason = "ok"
                    best_hand_landmarks = hl
                    break

        # Publish Bool
        out = Bool()
        out.data = meaningful
        self.pub_detected.publish(out)

        # Annotated image
        if self.publish_annotated:
            annotated = bgr.copy()

            # Draw ROI rectangle
            if self.roi_enabled and roi_rect is not None:
                x1, y1, x2, y2 = roi_rect
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(
                    annotated,
                    "ROI (down-middle)",
                    (x1 + 5, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            # Draw hand landmarks (for the best hand if present)
            if best_hand_landmarks is not None:
                self.mp_draw.draw_landmarks(
                    annotated,
                    best_hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                )

            # Status text
            if meaningful:
                label = "HAND: OK (ROI + DEPTH)"
                color = (0, 255, 0)
            elif hand_found:
                label = "HAND: REJECTED"
                color = (0, 165, 255)
            else:
                label = "NO HAND"
                color = (0, 0, 255)

            cv2.putText(
                annotated,
                label,
                (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                color,
                2,
                cv2.LINE_AA,
            )

            # Debug: ratio + depth + reason
            ratio_txt = f"ROI inside: {best_ratio*100:.0f}% (min {self.roi_min_inside_ratio*100:.0f}%)"
            if best_depth_m is None:
                depth_txt = f"Depth: n/a (need {self.min_depth_m*100:.0f}-{self.max_depth_m*100:.0f}cm)"
            else:
                depth_txt = f"Depth: {best_depth_m*100:.1f}cm (need {self.min_depth_m*100:.0f}-{self.max_depth_m*100:.0f}cm)"
            reason_txt = f"Reason: {best_reason}"

            cv2.putText(annotated, ratio_txt, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(annotated, depth_txt, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(annotated, reason_txt, (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            annot_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annot_msg.header = msg.header
            self.pub_annot.publish(annot_msg)


def main() -> None:
    rclpy.init()
    node = HandSemanticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()