#!/usr/bin/env python3
"""
flight_manager.py — Mission-level finite state machine for the E-gret drone.

Coordinates the full mission from arming through landing and docking. This is
the only node that makes top-level mission decisions; every other node
(energy_monitor, aruco_tracker, precision_land, docking_manager) reports
status to flight_manager and reacts to the state it publishes.

State sequence:
    IDLE -> ARMED -> MISSION_IN_PROGRESS -> RETURNING_TO_BASE
          -> PRECISION_LANDING -> DOCKING -> COMPLETED

Runs its FSM loop at 2 Hz.

Subscribes:
    /mavros/state              (mavros_msgs/State)         - armed flag, flight mode
    /mavros/extended_state     (mavros_msgs/ExtendedState) - landed/in-air status
    /energy_monitor/status     (std_msgs/String)           - "RTL_REQUEST" | "SAFE"
    /aruco/pose                (geometry_msgs/PoseStamped) - used only for freshness/timing
    /docking_manager/status    (std_msgs/String)           - "DOCKED" on success

Publishes:
    /flight_manager/state      (std_msgs/String)  - current FSM state, read by
                                                     precision_land & docking_manager
    /docking_manager/control   (std_msgs/String)  - "DOCKING" command to start dock sequence

Calls:
    /mavros/set_mode (mavros_msgs/SetMode) - to switch PX4 into AUTO.RTL / OFFBOARD
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode


class FlightManager(Node):
    """Mission FSM coordinating the E-gret return-to-base/land/dock sequence."""

    def __init__(self):
        super().__init__('flight_manager')

        # ================= FSM STATE =================
        # States: IDLE → ARMED → MISSION_IN_PROGRESS → RETURNING_TO_BASE
        #         → PRECISION_LANDING → LANDED → DOCKING → COMPLETED
        self.state = "IDLE"

        # ================= MAVROS STATE =================
        self.armed = False
        self.landed = False
        self.mode = ""
        self.px4_status = ""   # STANDBY / IN_AIR from ExtendedState

        # ================= INPUTS =================
        self.rtl_requested = False

        # ================= ARUCO =================
        self.last_aruco_time = None
        self.aruco_timeout = 0.5   # seconds — if older than this, marker is lost

        # ================= DOCKING =================
        self.docking_complete = False

        # ================= OFFBOARD SWITCH GUARD =================
        self.offboard_requested = False   # so we only request once

        # ================= SUBSCRIBERS =================
        self.create_subscription(State, '/mavros/state', self.mavros_state_cb, 10)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.ext_state_cb, 10)
        self.create_subscription(String, '/energy_monitor/status', self.energy_cb, 10)
        self.create_subscription(PoseStamped, '/aruco/pose', self.aruco_cb, 10)
        self.create_subscription(String, '/docking_manager/status', self.docking_cb, 10)

        # ================= PUBLISHERS =================
        self.state_pub = self.create_publisher(String, '/flight_manager/state', 10)
        self.docking_ctrl_pub = self.create_publisher(String, '/docking_manager/control', 10)

        # ================= SERVICE =================
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ================= TIMER =================
        self.create_timer(0.5, self.loop)

        self.get_logger().info("Flight Manager READY — State: IDLE")

    # =================================================
    # CALLBACKS
    # =================================================
    def mavros_state_cb(self, msg: State):
        self.armed = msg.armed
        self.mode = msg.mode

    def ext_state_cb(self, msg: ExtendedState):
        # landed_state: 0=UNDEFINED, 1=ON_GROUND, 2=IN_AIR, 3=TAKEOFF, 4=LANDING
        self.landed = (msg.landed_state == 1)
        self.px4_status = msg.landed_state

    def energy_cb(self, msg: String):
        if msg.data == "RTL_REQUEST":
            self.rtl_requested = True

    def aruco_cb(self, msg: PoseStamped):
        self.last_aruco_time = self.get_clock().now()

    def docking_cb(self, msg: String):
        if msg.data == "DOCKED":
            self.docking_complete = True

    # =================================================
    # HELPERS
    # =================================================
    def set_mode(self, mode: str):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetMode service unavailable")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f"Requested PX4 mode: {mode}")

    def aruco_fresh(self):
        """Returns True if ArUco pose was received within the timeout window."""
        if self.last_aruco_time is None:
            return False
        dt = (self.get_clock().now() - self.last_aruco_time).nanoseconds * 1e-9
        return dt < self.aruco_timeout

    def publish(self, state: str = None):
        if state:
            self.state = state
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    # =================================================
    # FSM LOOP (runs at 2 Hz)
    # =================================================
    def loop(self):

        # -----------------------------------------------
        # IDLE — waiting for the drone to arm
        # -----------------------------------------------
        if self.state == "IDLE":
            if self.armed:
                self.get_logger().info("Drone armed")
                self.publish("ARMED")

        # -----------------------------------------------
        # ARMED — waiting for takeoff / in-air confirmation
        # -----------------------------------------------
        elif self.state == "ARMED":
            # landed_state == 2 means IN_AIR
            if self.px4_status == 2:
                self.get_logger().info("Drone airborne — MISSION IN PROGRESS")
                self.publish("MISSION_IN_PROGRESS")

        # -----------------------------------------------
        # MISSION_IN_PROGRESS — monitor energy
        # -----------------------------------------------
        elif self.state == "MISSION_IN_PROGRESS":
            if self.rtl_requested:
                self.get_logger().warn(
                    "⚠️  ENERGY LEVEL MINIMUM — RETURNING TO BASE"
                )
                self.set_mode("AUTO.RTL")
                self.publish("RETURNING_TO_BASE")

        # -----------------------------------------------
        # RETURNING_TO_BASE — watch for ArUco marker
        # -----------------------------------------------
        elif self.state == "RETURNING_TO_BASE":
            if self.aruco_fresh():
                self.get_logger().warn(
                    "✅ MARKER DETECTED — SWITCHED TO OFFBOARD CONTROL — LANDING"
                )
                # Switch PX4 to OFFBOARD so precision_land setpoints take effect
                if not self.offboard_requested:
                    self.set_mode("OFFBOARD")
                    self.offboard_requested = True
                self.publish("PRECISION_LANDING")

        # -----------------------------------------------
        # PRECISION_LANDING — wait until physically on ground
        # -----------------------------------------------
        elif self.state == "PRECISION_LANDING":
            if self.landed and not self.armed:
                self.get_logger().info(
                    "🛬 DRONE LANDED — ESTABLISHING CONNECTION TO BATTERY SWAP STATION"
                )
                # Tell docking_manager it is its turn
                ctrl = String()
                ctrl.data = "DOCKING"
                self.docking_ctrl_pub.publish(ctrl)
                self.publish("DOCKING")

        # -----------------------------------------------
        # DOCKING — wait for confirmation from docking_manager
        # -----------------------------------------------
        elif self.state == "DOCKING":
            if self.docking_complete:
                self.get_logger().info("✅ Docking complete — MISSION COMPLETED")
                self.publish("COMPLETED")

        # -----------------------------------------------
        # COMPLETED — terminal state, nothing to do
        # -----------------------------------------------
        elif self.state == "COMPLETED":
            pass


def main():
    rclpy.init()
    node = FlightManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()