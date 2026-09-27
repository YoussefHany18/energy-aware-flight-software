#!/usr/bin/env python3
"""
aruco_tracker.py — Camera-based landing-pad pose estimation.

Detects a fiducial ArUco marker on the landing pad and solves for its 6-DoF
pose relative to the camera using solvePnP (SOLVEPNP_IPPE_SQUARE, since the
marker is a known-size planar square). Republishes the last known pose at a
fixed 20 Hz regardless of per-frame detection success, so precision_land
always has a continuous setpoint stream to keep PX4 in OFFBOARD mode -- a
gap here would drop the drone out of offboard control mid-landing.

Parameters:
    dictionary_id  = cv2.aruco.DICT_6X6_250
    marker_size    = 0.175 m
    image_topic    = /image_raw
    camera_yaml    = ~/picam_v2_calib.yaml  (intrinsics + distortion)
    publish_rate   = 20.0 Hz

Subscribes:
    <image_topic>      (sensor_msgs/Image) - raw camera feed

Publishes:
    /aruco/pose         (geometry_msgs/PoseStamped) - marker pose in camera_optical_frame
    /aruco/detected      (std_msgs/Bool)             - whether a marker is currently visible
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import os


class ArucoTracker(Node):
    """Detects an ArUco landing marker and streams its pose at a fixed rate."""

    def __init__(self):
        super().__init__('aruco_tracker')

        # ================= PARAMETERS =================
        self.declare_parameter('dictionary_id', cv2.aruco.DICT_6X6_250)
        self.declare_parameter('marker_size', 0.175)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_yaml', '~/picam_v2_calib.yaml')
        self.declare_parameter('publish_rate', 20.0)

        self.marker_size = self.get_parameter('marker_size').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_yaml = os.path.expanduser(self.get_parameter('camera_yaml').value)
        self.publish_rate = self.get_parameter('publish_rate').value

        # ================= CAMERA =================
        if not os.path.isfile(self.camera_yaml):
            self.get_logger().error(f"Missing calibration: {self.camera_yaml}")
            raise FileNotFoundError()

        with open(self.camera_yaml, 'r') as f:
            calib = yaml.safe_load(f)

        self.camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])

        # ================= ARUCO =================
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.get_parameter('dictionary_id').value)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())

        # ================= ROS =================
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.detect_pub = self.create_publisher(Bool, '/aruco/detected', 10)

        # ================= STATE =================
        self.last_pose = None
        self.marker_detected = False

        # ================= TIMER (20Hz guaranteed output) =================
        self.create_timer(1.0 / self.publish_rate, self.publish_loop)

        self.get_logger().info("ArucoTracker READY @ 20Hz OFFBOARD STREAM")

    # ================= IMAGE CALLBACK =================
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            self.marker_detected = False
            return

        self.marker_detected = True

        half = self.marker_size / 2.0
        obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        marker_corners = corners[0][0].astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return

        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_mat).as_quat()

        pose = PoseStamped()
        pose.header.frame_id = "camera_optical_frame"

        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        self.last_pose = pose

    # ================= 20Hz OUTPUT =================
    def publish_loop(self):

        msg = Bool()
        msg.data = self.marker_detected
        self.detect_pub.publish(msg)

        if self.last_pose is None:
            return

        self.last_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.last_pose)


def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()