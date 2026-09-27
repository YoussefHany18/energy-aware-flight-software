#!/usr/bin/env python3
"""
energy_monitor.py — Control-Barrier-Function-based return-to-launch trigger.

Instead of a fixed low-battery threshold, this node estimates the energy
needed to return home (travel + climb) from live telemetry, adds a safety
margin and a fixed critical reserve, and evaluates a Control Barrier
Function (CBF) each tick: h = remaining_energy - needed - margin - reserve,
checked against (h_dot + gamma * h) < 0. This flags a *closing* energy
margin, not just a low one, and only fires after `violation_threshold`
consecutive violations to reject single-sample sensor noise. Runs at 1 Hz,
active only while flight_manager reports MISSION_IN_PROGRESS.

Key parameters:
    battery_capacity_wh = 58.0   - pack capacity
    critical_ratio       = 0.22  - fixed reserve fraction, never spent
    gamma                = 0.5   - CBF decay rate
    violation_threshold  = 2     - consecutive violations before RTL fires

Subscribes:
    /mavros/battery                 (sensor_msgs/BatteryState)   - % and V/I for power draw
    /mavros/local_position/pose     (geometry_msgs/PoseStamped)  - current position
    /mavros/home_position/home      (mavros_msgs/HomePosition)   - home position
    /flight_manager/state           (std_msgs/String)            - gates the loop to MISSION_IN_PROGRESS

Publishes:
    /energy_monitor/status          (std_msgs/String) - "SAFE" or "RTL_REQUEST" (sent once)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import HomePosition
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class EnergyMonitor(Node):
    """CBF-based energy monitor that triggers a one-shot RTL_REQUEST."""

    def __init__(self):
        super().__init__('energy_monitor')

        # ================= PARAMETERS =================
        self.battery_capacity_wh = 58.0
        self.critical_ratio = 0.22
        self.E_crit = self.battery_capacity_wh * self.critical_ratio
        self.return_speed = 6.0          # m/s estimated return speed

        # ================= POWER TRACKING =================
        self.voltage = None
        self.current = None
        self.power_buffer = []
        self.buffer_size = 10
        self.power_avg = 360.0           # fallback average power (W)

        # ================= STATE =================
        self.battery_wh = None
        self.position = None
        self.home_position = None
        self.flight_state = "IDLE"       # synced from flight_manager

        # ================= CBF =================
        self.gamma = 0.5
        self.h_previous = None
        self.violation_counter = 0
        self.violation_threshold = 2     # consecutive violations before RTL

        # ================= RTL GUARD =================
        # We only ever send this once — flight_manager takes it from there
        self.rtl_sent = False

        # ================= QoS (MAVROS topics need BEST_EFFORT) =================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ================= SUBSCRIBERS =================
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)
        self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_cb, qos)
        self.create_subscription(String, '/flight_manager/state', self.state_cb, 10)

        # ================= PUBLISHERS =================
        # Sends either "SAFE" or "RTL_REQUEST" to flight_manager
        self.status_pub = self.create_publisher(String, '/energy_monitor/status', 10)

        self.create_timer(1.0, self.loop)

        self.get_logger().info("Energy Monitor READY")

    # =================================================
    # CALLBACKS
    # =================================================
    def battery_cb(self, msg: BatteryState):
        if msg.percentage >= 0.0:
            self.battery_wh = msg.percentage * self.battery_capacity_wh

        self.voltage = msg.voltage
        self.current = msg.current

        if self.voltage and self.current and self.voltage > 0 and self.current != 0:
            p = abs(self.voltage * self.current)
            self.power_buffer.append(p)
            if len(self.power_buffer) > self.buffer_size:
                self.power_buffer.pop(0)
            self.power_avg = sum(self.power_buffer) / len(self.power_buffer)

    def pose_cb(self, msg: PoseStamped):
        self.position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )

    def home_cb(self, msg):
        self.home_position = (
            msg.position.x,
            msg.position.y,
            msg.position.z
        )

    def state_cb(self, msg: String):
        self.flight_state = msg.data

    # =================================================
    # ENERGY MODEL
    # =================================================
    def compute_energy_needed(self, d, z, hz):
        """
        Estimate energy (Wh) needed to return home from current position.
        Accounts for horizontal travel time and any required altitude gain.
        """
        t_travel = d / self.return_speed
        e_travel = (self.power_avg * t_travel) / 3600.0

        dh = max(0.0, z - hz)          # only penalise if we need to climb
        e_climb = (1.8 * 9.81 * dh) / 3600.0

        return e_travel + e_climb

    def safety_margin(self, e):
        """40 % buffer on top of estimated energy + fixed 5 Wh pad."""
        return 0.4 * e + 5.0

    # =================================================
    # MAIN LOOP (1 Hz)
    # =================================================
    def loop(self):

        # ---- Only run during active mission ----
        if self.flight_state != "MISSION_IN_PROGRESS":
            # Reset CBF state so it starts fresh when mission begins
            self.violation_counter = 0
            self.h_previous = None
            return

        # ---- Don't keep re-sending after RTL already requested ----
        if self.rtl_sent:
            return

        # ---- Need all data before computing ----
        if self.battery_wh is None or self.position is None or self.home_position is None:
            return

        x, y, z = self.position
        hx, hy, hz = self.home_position

        d = math.sqrt((x - hx)**2 + (y - hy)**2 + (z - hz)**2)

        e = self.compute_energy_needed(d, z, hz)
        m = self.safety_margin(e)

        # CBF barrier: h > 0 means we still have enough energy to return safely
        h = self.battery_wh - e - m - self.E_crit

        # Initialise on first tick
        if self.h_previous is None:
            self.h_previous = h

        h_dot = h - self.h_previous
        cbf = h_dot + self.gamma * h

        self.h_previous = h

        # ---- Violation counting ----
        if cbf < 0:
            self.violation_counter += 1
            self.get_logger().warn(
                f"CBF violation {self.violation_counter}/{self.violation_threshold} "
                f"| h={h:.2f} Wh | battery={self.battery_wh:.2f} Wh"
            )
        else:
            self.violation_counter = 0

        # ---- Publish status ----
        status = String()

        if self.violation_counter >= self.violation_threshold:
            status.data = "RTL_REQUEST"
            self.rtl_sent = True          # send once, flight_manager owns it now
            self.get_logger().warn("🔴 RTL_REQUEST sent to Flight Manager")
        else:
            status.data = "SAFE"

        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = EnergyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()