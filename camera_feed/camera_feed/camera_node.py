#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraFeedNode(Node):

    def __init__(self):
        super().__init__('camera_feed_node')

        self.bridge = CvBridge()

        # Subscriber to v4l2_camera output
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',   # default topic from v4l2_camera
            self.image_callback,
            10
        )

        # Publisher for processed image (optional but recommended)
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        self.get_logger().info("Camera Feed Node Started - Subscribed to /image_raw")

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ---------------------------
        # PROCESSING SECTION
        # ---------------------------

        # Example processing (simple overlay text)
        cv2.putText(
            frame,
            "Drone Camera Active",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        # ---------------------------
        # Convert back to ROS message
        # ---------------------------
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        processed_msg.header = msg.header

        self.publisher.publish(processed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()