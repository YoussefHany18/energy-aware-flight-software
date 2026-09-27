import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2
import os
from datetime import datetime
import tflite_runtime.interpreter as tflite


class AIDetectionNode(Node):
    def __init__(self):
        super().__init__('ai_detection_node')

        self.get_logger().info("Bad Crop Detection Node Started")

        self.bridge = CvBridge()

        # ---------------- MODEL ----------------
        self.interpreter = tflite.Interpreter(
            model_path='/home/drone/drone_ws/src/ai_detection/ai_detection/best_float32.tflite'
        )
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.h = self.input_details[0]['shape'][1]
        self.w = self.input_details[0]['shape'][2]

        # ---------------- PX4 LOCAL POSITION ----------------
        self.local_pose = None

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_position_callback,
            10
        )

        # ---------------- CAMERA ----------------
        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # ---------------- OUTPUT ----------------
        self.pub = self.create_publisher(String, '/ai_output', 10)

        # ---------------- STORAGE ----------------
        self.save_dir = "/home/drone/bad_crops"
        os.makedirs(self.save_dir, exist_ok=True)

    # ================= PX4 CALLBACK =================
    def local_position_callback(self, msg):
        self.local_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    # ================= PREPROCESS =================
    def preprocess(self, frame):
        frame = cv2.resize(frame, (self.w, self.h))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = frame.astype(np.float32) / 255.0
        frame = np.expand_dims(frame, axis=0)
        return frame

    # ================= SAVE DATA =================
    def save_data(self, frame, score):

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        img_path = f"{self.save_dir}/img_{timestamp}.jpg"
        cv2.imwrite(img_path, frame)

        # ---------------- POSITION HANDLING ----------------
        if self.local_pose is None:
            pose_text = "PX4_NOT_CONNECTED"
        else:
            x, y, z = self.local_pose
            pose_text = f"x={x:.3f}, y={y:.3f}, z={z:.3f}"

        # ---------------- LOG FILE ----------------
        log_path = f"{self.save_dir}/log.txt"

        with open(log_path, "a") as f:
            f.write(f"{img_path}, score={score:.3f}, {pose_text}\n")

        self.get_logger().info(
            f"Saved bad crop | score={score:.3f} | {pose_text}"
        )

    # ================= INFERENCE =================
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        input_data = self.preprocess(frame)

        self.interpreter.set_tensor(
            self.input_details[0]['index'],
            input_data
        )

        self.interpreter.invoke()

        output = self.interpreter.get_tensor(
            self.output_details[0]['index']
        )

        # ---------------- YOLO SCORE ----------------
        output = output[0]
        scores = output[4]
        score = np.max(scores)

        self.pub.publish(String(data=f"Score: {score:.3f}"))

        self.get_logger().info(f"MAX SCORE: {score:.3f}")

        # ---------------- SAVE CONDITION ----------------
        if score > 0.85:
            self.save_data(frame, score)


def main(args=None):
    rclpy.init(args=args)
    node = AIDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()