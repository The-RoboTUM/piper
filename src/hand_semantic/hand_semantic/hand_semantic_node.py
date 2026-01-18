#!/usr/bin/env python3
from __future__ import annotations

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
    Semantic hand detector using MediaPipe Hands.

    Subscribes:
      - RGB image topic (default: /camera/camera/color/image_rect_raw)

    Publishes:
      - /hand/semantic_detected (std_msgs/Bool)
      - /hand/annotated_image  (sensor_msgs/Image)  [BGR8]
    """

    def __init__(self) -> None:
        super().__init__("hand_semantic_node")

        # Parameters
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_rect_raw")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)
        self.declare_parameter("max_num_hands", 1)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.min_det_conf = float(self.get_parameter("min_detection_confidence").value)
        self.min_trk_conf = float(self.get_parameter("min_tracking_confidence").value)
        self.max_num_hands = int(self.get_parameter("max_num_hands").value)

        # ROS I/O
        self.bridge = CvBridge()
        self.pub_detected = self.create_publisher(Bool, "/hand/semantic_detected", 10)
        self.pub_annot = self.create_publisher(Image, "/hand/annotated_image", 10)

        self.sub = self.create_subscription(
            Image, self.rgb_topic, self.on_rgb, qos_profile_sensor_data
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
            f"HandSemanticNode up. Subscribing: {self.rgb_topic} | Publishing: /hand/semantic_detected"
        )

    def on_rgb(self, msg: Image) -> None:
        # Convert ROS Image -> OpenCV BGR
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # MediaPipe expects RGB
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        results = self.hands.process(rgb)
        hand_found = bool(results.multi_hand_landmarks)

        out = Bool()
        out.data = hand_found
        self.pub_detected.publish(out)

        if self.publish_annotated:
            annotated = bgr.copy()
            if hand_found:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        annotated,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                    )
                cv2.putText(
                    annotated,
                    "HAND",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    annotated,
                    "NO HAND",
                    (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            annot_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annot_msg.header = msg.header  # keep timestamp/frame_id
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
