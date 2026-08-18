#!/usr/bin/env python3
"""
sulg_ros2_node.py  –  ROS 2 Node for ESP32 Sulg Low-Level Motion Controller
For Ubuntu 24.04 LTS / ROS 2 (Jazzy / Humble) on Raspberry Pi 5.

Subscriptions:
  /cmd_vel  (geometry_msgs/msg/Twist)  – High-level velocity commands

Publications:
  /odom     (nav_msgs/msg/Odometry)    – 2D Dead-Reckoning Odometry for Nav2 & SLAM
  TF: odom -> base_link
"""

import math
import serial
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class SulgRos2Node(Node):
    def __init__(self):
        super().__init__('sulg_ros2_node')
        self.ser = None
        self.running = False

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_center', 110)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.servo_center = self.get_parameter('servo_center').value

        # Serial Connection
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {self.port} @ {self.baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            return

        # ROS 2 Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Thread for reading Serial Telemetry from ESP32
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        """Convert ROS 2 /cmd_vel (linear.x, angular.z) into ESP32 Drive Command."""
        linear_x = msg.linear.x    # m/s
        angular_z = msg.angular.z  # rad/s

        # Map linear velocity (m/s) to ESP32 Motor Speed (-255..255)
        speed = int(linear_x * 255.0)
        speed = max(-255, min(255, speed))

        # Map angular.z (rad/s) to steering deflection degrees
        steer_deflection_deg = math.degrees(angular_z)
        steer_deflection_deg = max(-30.0, min(30.0, steer_deflection_deg))

        # Map to servo PWM (50..170, center 110)
        steer_angle = int(round(self.servo_center + (steer_deflection_deg / 30.0) * 60.0))
        steer_angle = max(50, min(170, steer_angle))

        # Send command to ESP32: D <speed> <steer_angle>
        cmd = f"D {speed} {steer_angle}\n"
        if self.ser and self.ser.is_open:
            self.ser.write(cmd.encode('utf-8'))

    def _read_serial_loop(self):
        """Read 20 Hz telemetry from ESP32 and publish to ROS 2 /odom & TF."""
        while rclpy.ok() and self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) >= 7:
                        x_mm     = float(parts[1])
                        y_mm     = float(parts[2])
                        yaw_deg  = float(parts[3])
                        speed_mms= float(parts[5])

                        # Convert mm -> meters, deg -> radians
                        x_m = x_mm / 1000.0
                        y_m = y_mm / 1000.0
                        yaw_rad = yaw_deg * (math.pi / 180.0)
                        speed_ms = speed_mms / 1000.0

                        current_time = self.get_clock().now().to_msg()

                        # Quaternion orientation from yaw
                        qz = math.sin(yaw_rad / 2.0)
                        qw = math.cos(yaw_rad / 2.0)

                        # Publish TF (odom -> base_link)
                        t = TransformStamped()
                        t.header.stamp = current_time
                        t.header.frame_id = 'odom'
                        t.child_frame_id = 'base_link'
                        t.transform.translation.x = x_m
                        t.transform.translation.y = y_m
                        t.transform.translation.z = 0.0
                        t.transform.rotation.z = qz
                        t.transform.rotation.w = qw
                        self.tf_broadcaster.sendTransform(t)

                        # Publish ROS 2 Odometry Topic
                        odom = Odometry()
                        odom.header.stamp = current_time
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_link'

                        odom.pose.pose.position.x = x_m
                        odom.pose.pose.position.y = y_m
                        odom.pose.pose.orientation.z = qz
                        odom.pose.pose.orientation.w = qw

                        odom.twist.twist.linear.x = speed_ms
                        self.odom_pub.publish(odom)

            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = SulgRos2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if node.ser and node.ser.is_open:
            node.ser.write(b"S\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
