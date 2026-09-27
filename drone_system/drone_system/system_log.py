#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState

from datetime import datetime
import os


class SystemLogger(Node):

    def __init__(self):
        super().__init__('system_logger')

        # ================= LOG FILE =================
        self.log_file = os.path.expanduser("~/system_log.txt")
        with open(self.log_file, "w") as f:
            f.write("==== SYSTEM LOG STARTED ====\n")

        # ================= SUBSCRIBERS =================
        self.create_subscription(String, '/flight_manager/state', self.flight_cb, 10)
        self.create_subscription(String, '/energy_monitor/status', self.energy_cb, 10)
        self.create_subscription(String, '/precision_land/status', self.precision_cb, 10)
        self.create_subscription(String, '/docking_manager/status', self.docking_cb, 10)

        self.create_subscription(PoseStamped, '/aruco/pose', self.aruco_cb, 10)

        self.create_subscription(State, '/mavros/state', self.mavros_cb, 10)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.ext_cb, 10)

        self.get_logger().info("SYSTEM LOGGER ACTIVE")

    # ================= UTILITY =================
    def log(self, tag, msg):

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{timestamp}] [{tag}] {msg}"

        print(line)

        with open(self.log_file, "a") as f:
            f.write(line + "\n")

    # ================= CALLBACKS =================
    def flight_cb(self, msg: String):
        self.log("FLIGHT_STATE", msg.data)

    def energy_cb(self, msg: String):
        self.log("ENERGY", msg.data)

    def precision_cb(self, msg: String):
        self.log("PRECISION", msg.data)

    def docking_cb(self, msg: String):
        self.log("DOCKING", msg.data)

    def aruco_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.log("ARUCO",
                 f"x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}")

    def mavros_cb(self, msg: State):
        self.log("MAVROS",
                 f"armed={msg.armed}, mode={msg.mode}")

    def ext_cb(self, msg: ExtendedState):
        self.log("EXT_STATE",
                 f"landed_state={msg.landed_state}")


def main():
    rclpy.init()
    node = SystemLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()