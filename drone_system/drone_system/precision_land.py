#!/usr/bin/env python3
"""
precision_land.py — Vision-guided final approach and landing trigger.

Active only while flight_manager reports PRECISION_LANDING. Takes ArUco
poses, applies a correction factor to the estimated altitude (calibrated
against measured landing error), and streams the corrected setpoint to
PX4's local setpoint topic at 20 Hz to hold OFFBOARD mode. Once altitude
drops below land_threshold, calls AUTO.LAND exactly once (latched via
landing_triggered) rather than re-triggering on every subsequent pose.

Parameters:
    correction_factor = 2.0 / 1.47  - Z-axis correction from measured landing error
    land_threshold    = 0.7 m       - altitude below which AUTO.LAND is triggered
    publish_rate      = 20.0 Hz

Subscribes:
    /aruco/pose             (geometry_msgs/PoseStamped) - marker pose from aruco_tracker
    /flight_manager/state   (std_msgs/String)           - gates all activity to PRECISION_LANDING

Publishes:
    /mavros/setpoint_position/local (geometry_msgs/PoseStamped) - corrected landing setpoint
    /precision_land/status           (std_msgs/String)          - "ACTIVE" | "LANDING"

Calls:
    /mavros/set_mode (mavros_msgs/SetMode) - AUTO.LAND once threshold is crossed
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
import copy


class PrecisionLand(Node):
    """Streams corrected ArUco-based setpoints and triggers AUTO.LAND once close enough."""

    def __init__(self):
        super().__init__('precision_land')

        # ================= PARAMETERS =================
        self.declare_parameter('correction_factor', 2.0 / 1.47)
        self.declare_parameter('land_threshold', 0.7)
        self.declare_parameter('publish_rate', 20.0)

        self.correction_factor = self.get_parameter('correction_factor').value
        self.land_threshold = self.get_parameter('land_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ================= STATE =================
        # Activated purely by /flight_manager/state == "PRECISION_LANDING"
        self.flight_state = "IDLE"
        self.landing_triggered = False

        # ================= INTERNAL =================
        # Holds the last known ArUco setpoint.
        # Published at 20 Hz even if ArUco is temporarily lost,
        # so OFFBOARD mode never drops due to missing setpoints.
        self.last_setpoint = None

        # ================= SUBSCRIBERS =================
        self.create_subscription(PoseStamped, '/aruco/pose', self.aruco_callback, 10)
        self.create_subscription(String, '/flight_manager/state', self.state_callback, 10)

        # ================= PUBLISHERS =================
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/precision_land/status',
            10
        )

        # ================= MAVROS =================
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ================= LOOP (20 Hz) =================
        self.create_timer(1.0 / self.publish_rate, self.loop)

        self.get_logger().info("PrecisionLand READY — waiting for PRECISION_LANDING state")

    # =================================================
    # STATE INPUT (FROM FLIGHT MANAGER)
    # =================================================
    def state_callback(self, msg: String):
        self.flight_state = msg.data

        # Reset landing trigger if we somehow re-enter the state
        if msg.data != "PRECISION_LANDING":
            self.landing_triggered = False

    # =================================================
    # ARUCO INPUT
    # =================================================
    def aruco_callback(self, msg: PoseStamped):

        # Only process ArUco poses when we are in precision landing
        if self.flight_state != "PRECISION_LANDING":
            return

        if self.landing_triggered:
            return

        setpoint = copy.deepcopy(msg)

        original_z = setpoint.pose.position.z
        setpoint.pose.position.z *= self.correction_factor

        self.last_setpoint = setpoint

        self.get_logger().info(
            f"[PrecisionLand] ArUco Z {original_z:.2f} → corrected {setpoint.pose.position.z:.2f} m"
        )

        # LAND CONDITION: close enough to ground
        if setpoint.pose.position.z <= self.land_threshold:
            self.trigger_land()

    # =================================================
    # LAND COMMAND
    # =================================================
    def trigger_land(self):

        if self.landing_triggered:
            return

        self.landing_triggered = True

        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.land_response)

        self.publish_status("LANDING")

        self.get_logger().warn("AUTO.LAND triggered by PrecisionLand")

    def land_response(self, future):
        try:
            if future.result().mode_sent:
                self.get_logger().info("AUTO.LAND accepted by PX4")
            else:
                self.get_logger().error("AUTO.LAND rejected by PX4")
        except Exception as e:
            self.get_logger().error(f"AUTO.LAND service error: {e}")

    # =================================================
    # MAIN LOOP — 20 Hz OFFBOARD SETPOINT STREAM
    # =================================================
    def loop(self):

        # Only stream when flight manager says so
        if self.flight_state != "PRECISION_LANDING":
            return

        # Always publish last known setpoint at 20 Hz to maintain OFFBOARD mode,
        # even if ArUco detection has a temporary gap
        if self.last_setpoint is not None and not self.landing_triggered:
            self.last_setpoint.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.last_setpoint)
            self.publish_status("ACTIVE")

    # =================================================
    # STATUS
    # =================================================
    def publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = PrecisionLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()