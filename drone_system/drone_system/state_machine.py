#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import StatusText


class DroneStateMachine(Node):

    def __init__(self):
        super().__init__('drone_state_machine')

        # ================= STATE =================
        self.state = "IDLE"

        # ================= PRIORITY (CRITICAL FIX) =================
        self.priority = {
            "IDLE": 0,
            "MISSION": 1,
            "HOLD_HOME": 2,
            "PRECISION_LANDING": 3,
            "DOCKING": 4,
            "DOCKED": 5
        }

        # ================= SUBS =================
        self.create_subscription(String, '/mission_state', self.mission_cb, 10)
        self.create_subscription(String, '/rtl_status', self.rtl_cb, 10)
        self.create_subscription(String, '/precision_land_status', self.precision_cb, 10)
        self.create_subscription(String, '/docking_status', self.docking_cb, 10)

        # ================= OUTPUT =================
        self.state_pub = self.create_publisher(String, '/drone_state', 10)
        self.qgc_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        self.create_timer(0.5, self.loop)

        self.get_logger().info("STATE MACHINE READY (SAFE MODE)")

    # ================= SAFE STATE TRANSITION =================
    def set_state(self, new_state: str):

        # ❌ prevent downgrade
        if self.priority[new_state] < self.priority[self.state]:
            return

        if new_state == self.state:
            return

        self.state = new_state

        self.get_logger().info(f"STATE → {self.state}")

        # ================= ROS OUTPUT =================
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

        # ================= QGC OUTPUT =================
        qgc = StatusText()
        qgc.severity = 6
        qgc.text = f"FSM: {self.state}"
        self.qgc_pub.publish(qgc)

    # ================= CALLBACKS =================
    def mission_cb(self, msg):
        if msg.data == "ACTIVE":
            self.set_state("MISSION")

    def rtl_cb(self, msg):
        if msg.data == "TRIGGERED":
            self.set_state("HOLD_HOME")

        # 🔥 CRITICAL FIX: RTL → landing pipeline start
        elif msg.data == "READY_LAND":
            self.set_state("PRECISION_LANDING")

    def precision_cb(self, msg):
        if msg.data == "ACTIVE":
            self.set_state("PRECISION_LANDING")

        elif msg.data == "LANDED":
            self.set_state("DOCKING")

    def docking_cb(self, msg):
        if msg.data == "DOCKED":
            self.set_state("DOCKED")

    # ================= LOOP =================
    def loop(self):
        pass


def main():
    rclpy.init()
    node = DroneStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()