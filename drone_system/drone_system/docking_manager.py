#!/usr/bin/env python3
"""
docking_manager.py — Ground-side docking sequence with the battery-swap station.

Waits for an explicit DOCKING command from flight_manager AND independent
confirmation (via MAVROS) that the drone is landed and disarmed before
acting -- both conditions must hold, since neither signal alone is a
reliable "safe to dock" guarantee. On trigger, opens the docking connector
via a GPIO-driven servo and negotiates the swap over a Bluetooth serial
link with the station using a simple DOCK_REQUEST -> CONFIRMED handshake.

Hardware:
    SERVO_PIN = 18 (BCM), PWM-driven, logical angle inverted in move_servo()
    Bluetooth: /dev/rfcomm0 @ 115200 baud

Subscribes:
    /flight_manager/state       (std_msgs/String)            - gates activity to DOCKING
    /docking_manager/control    (std_msgs/String)            - "DOCKING" start command
    /mavros/state               (mavros_msgs/State)          - disarmed check
    /mavros/extended_state      (mavros_msgs/ExtendedState)  - landed check

Publishes:
    /docking_manager/status     (std_msgs/String) - "DOCKING" | "DOCKED" | "SHUTDOWN" | "DOCKING_FAILED"
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState
import RPi.GPIO as GPIO
import time
import serial


SERVO_PIN = 18
OPEN = 90
CLOSED = 0


class DockingManager(Node):
    """Drives the docking servo and negotiates the battery swap over Bluetooth."""

    def __init__(self):
        super().__init__('docking_manager')

        # ================= STATE =================
        self.flight_state = "IDLE"

        # Control gate — only act when FlightManager explicitly says "DOCKING"
        self.docking_commanded = False

        self.drone_landed = False
        self.drone_disarmed = False

        self.docking_started = False

        # ================= ROS =================
        self.create_subscription(String, '/flight_manager/state', self.state_cb, 10)
        self.create_subscription(String, '/docking_manager/control', self.control_cb, 10)
        self.create_subscription(State, '/mavros/state', self.mavros_state_cb, 10)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.ext_state_cb, 10)

        self.status_pub = self.create_publisher(String, '/docking_manager/status', 10)

        # ================= SERVO =================
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo = GPIO.PWM(SERVO_PIN, 50)
        self.servo.start(0)
        self.move_servo(CLOSED)

        # ================= BLUETOOTH =================
        self.bt = None
        self.bt_ready = False

        self.create_timer(0.5, self.loop)

        self.get_logger().info("Docking Manager READY — waiting for DOCKING command")

    # =================================================
    # CALLBACKS
    # =================================================
    def state_cb(self, msg: String):
        self.flight_state = msg.data

    def control_cb(self, msg: String):
        if msg.data == "DOCKING":
            self.docking_commanded = True
            self.get_logger().info("Docking command received from Flight Manager")

    def mavros_state_cb(self, msg: State):
        self.drone_disarmed = not msg.armed

    def ext_state_cb(self, msg: ExtendedState):
        self.drone_landed = (msg.landed_state == 1)

    # =================================================
    # MAIN LOOP (2 Hz)
    # =================================================
    def loop(self):

        if self.flight_state != "DOCKING":
            return

        if not self.docking_commanded:
            return

        if not (self.drone_landed and self.drone_disarmed):
            return

        if not self.docking_started:
            self.start_docking()

    # =================================================
    # DOCKING SEQUENCE
    # =================================================
    def start_docking(self):

        self.docking_started = True
        self.publish("DOCKING")
        self.get_logger().info("Docking sequence started")

        self.move_servo(OPEN)

        if self.connect_bt():
            resp = self.send_bt("DOCK_REQUEST")

            if resp and "CONFIRMED" in resp:
                self.get_logger().info("✅ Dock confirmed by battery swap station")
                self.publish("DOCKED")
                time.sleep(2)
                self.publish("SHUTDOWN")
            else:
                self.get_logger().error("❌ No confirmation from battery swap station")
                self.publish("DOCKING_FAILED")
        else:
            self.get_logger().error("❌ Bluetooth connection to station failed")
            self.publish("DOCKING_FAILED")

    # =================================================
    # SERVO (FIXED: INVERTED)
    # =================================================
    def move_servo(self, angle):
        inverted = 90 - angle   # 🔥 FIX: reverses direction

        duty = 2 + (inverted / 18)
        self.servo.ChangeDutyCycle(duty)
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0)

        self.get_logger().info(f"Servo → logical {angle}° | physical {inverted}°")

    # =================================================
    # BLUETOOTH
    # =================================================
    def connect_bt(self):
        if self.bt:
            return True
        try:
            self.bt = serial.Serial('/dev/rfcomm0', 115200, timeout=2)
            time.sleep(1)
            self.bt_ready = True
            self.get_logger().info("Bluetooth connected to station")
            return True
        except Exception as e:
            self.get_logger().warn(f"Bluetooth connection failed: {e}")
            return False

    def send_bt(self, msg: str):
        if not self.bt_ready:
            return None
        try:
            self.bt.write((msg + "\n").encode())
            time.sleep(0.5)
            if self.bt.in_waiting:
                return self.bt.readline().decode().strip()
        except Exception as e:
            self.get_logger().warn(f"Bluetooth send error: {e}")
        return None

    # =================================================
    # STATUS
    # =================================================
    def publish(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

    # =================================================
    # CLEANUP
    # =================================================
    def destroy_node(self):
        if self.bt:
            self.bt.close()
        self.servo.stop()
        GPIO.cleanup()
        super().destroy_node()


def main():
    rclpy.init()
    node = DockingManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()