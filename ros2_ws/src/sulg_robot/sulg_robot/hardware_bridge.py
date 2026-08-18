#!/usr/bin/env python3
"""
hardware_bridge.py – ROS 2 Jazzy Hardware Interface Node for ESP32 Motion Controller
Subscribes: /cmd_vel (geometry_msgs/msg/Twist)
Publishes: /odom (nav_msgs/msg/Odometry), TF: odom -> base_footprint, buttons & diagnostics
"""

import os
import math
import time
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int32
from sensor_msgs.msg import JointState, Imu
from std_srvs.srv import Trigger, Empty
from tf2_ros import TransformBroadcaster

class SulgHardwareBridge(Node):
    def __init__(self):
        super().__init__('sulg_hardware_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_56BA018173-if00')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_center', 110)
        self.declare_parameter('steer_angle_scale', 30.0) # Deflection degrees per rad/s
        self.declare_parameter('max_steer_deflection', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('invert_odom_x', True)
        self.declare_parameter('invert_odom_y', True)
        self.declare_parameter('invert_yaw', True)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.servo_center = self.get_parameter('servo_center').value
        self.steer_scale = self.get_parameter('steer_angle_scale').value
        self.max_steer_def = self.get_parameter('max_steer_deflection').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert_odom_x = self.get_parameter('invert_odom_x').value
        self.invert_odom_y = self.get_parameter('invert_odom_y').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        self.steer_angle_rad = 0.0
        self.wheel_pos_rad = 0.0
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.speed_mps = 0.0
        self.dist_mm = 0.0
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.last_pub_time = time.time()

        # Fallback to standard ACM0 if port not found
        if not os.path.exists(self.port) and os.path.exists('/dev/ttyACM0'):
            self.port = '/dev/ttyACM0'

        # Serial Connection
        self.ser = None
        self.running = True
        self.connect_serial()

        # ROS 2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.btn1_pub = self.create_publisher(Bool, '/esp32/button1', 10)
        self.btn2_pub = self.create_publisher(Bool, '/esp32/button2', 10)
        self.diag_pub = self.create_publisher(String, '/esp32/diagnostics', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Service Client for starting LiDAR motor (one-time trigger)
        self.start_lidar_cli = self.create_client(Empty, '/start_motor')
        self.motor_started = False
        self.motor_timer = self.create_timer(3.0, self._try_start_lidar_motor)

        # ROS 2 Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ROS 2 Services
        self.reset_srv = self.create_service(Trigger, '/reset_odom', self.reset_odom_callback)
        self.estop_srv = self.create_service(Trigger, '/emergency_stop', self.estop_callback)

        # 20Hz Reliable Telemetry & TF Publisher Timer
        self.pub_timer = self.create_timer(0.05, self._publish_telemetry)

        # Serial Background Reader Thread
        self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f"[READY] Sulg Hardware Bridge active on {self.port} @ {self.baudrate} baud.")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"[SUCCESS] Connected to ESP32 on {self.port}")
        except Exception as e:
            self.get_logger().warn(f"[WARN] Could not connect to {self.port}: {e}. Retrying in background...")

    def cmd_vel_callback(self, msg: Twist):
        """Convert standard ROS 2 Twist (/cmd_vel) to ESP32 'D <speed> <steer>' packet."""
        linear_x = msg.linear.x    # Forward velocity in m/s
        angular_z = msg.angular.z  # Steering angular velocity in rad/s

        # Map linear.x (0..1.5 m/s) to ESP32 Motor PWM (-255..255)
        # 1.5 m/s approx = 255 PWM
        speed_pwm = int((linear_x / 1.5) * 255.0)
        speed_pwm = max(-255, min(255, speed_pwm))

        # Map angular.z (rad/s) to steering deflection degrees
        steer_deflection_deg = math.degrees(angular_z)
        steer_deflection_deg = max(-self.max_steer_def, min(self.max_steer_def, steer_deflection_deg))

        # Map to servo PWM (50..170, center 110)
        steer_pwm = int(round(self.servo_center + (steer_deflection_deg / self.max_steer_def) * 60.0))
        steer_pwm = max(50, min(170, steer_pwm))

        self.cmd_linear_x = linear_x
        self.cmd_angular_z = angular_z
        cmd = f"D {speed_pwm} {steer_pwm}\n"
        self._write_serial(cmd)

    def _write_serial(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")

    def reset_odom_callback(self, request, response):
        self._write_serial("R\n")
        response.success = True
        response.message = "ESP32 Odometry and Yaw reset to 0.0"
        self.get_logger().info("Service called: Odometry reset.")
        return response

    def estop_callback(self, request, response):
        self._write_serial("S\n")
        response.success = True
        response.message = "Emergency stop sent to ESP32."
        self.get_logger().warn("Service called: Emergency Stop!")
        return response

    def _serial_read_loop(self):
        """Continuously parse 20Hz ODOM telemetry from ESP32."""
        while self.running and rclpy.ok():
            if not self.ser or not self.ser.is_open:
                time.sleep(1.0)
                self.connect_serial()
                continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("ODOM,"):
                    continue

                parts = line.split(',')
                if len(parts) < 9:
                    continue

                x_mm      = float(parts[1])
                y_mm      = float(parts[2])
                yaw_deg   = float(parts[3])
                dist_mm   = float(parts[4])
                speed_mms = float(parts[5])
                sensors_ok= bool(int(parts[6]))
                btn1      = bool(int(parts[7]))
                btn2      = bool(int(parts[8]))

                # Metric conversions state update (with direction inversion support)
                raw_x = x_mm / 1000.0
                raw_y = y_mm / 1000.0
                raw_yaw = math.radians(yaw_deg)
                raw_speed = speed_mms / 1000.0

                self.x_m = -raw_x if self.invert_odom_x else raw_x
                self.y_m = -raw_y if self.invert_odom_y else raw_y
                self.yaw_rad = -raw_yaw if self.invert_yaw else raw_yaw
                self.speed_mps = -raw_speed if self.invert_odom_x else raw_speed
                self.dist_mm = dist_mm

                # 4. Publish Button Triggers
                if btn1:
                    b1_msg = Bool()
                    b1_msg.data = True
                    self.btn1_pub.publish(b1_msg)

                if btn2:
                    b2_msg = Bool()
                    b2_msg.data = True
                    self.btn2_pub.publish(b2_msg)

            except Exception as e:
                time.sleep(0.01)

    def _publish_telemetry(self):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw_rad / 2.0)
        qw = math.cos(self.yaw_rad / 2.0)

        # 1. Publish TF (odom -> base_footprint)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = self.x_m
            tf.transform.translation.y = self.y_m
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = 0.0
            tf.transform.rotation.y = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        # 2. Publish /odom Topic
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x_m
        odom.pose.pose.position.y = self.y_m
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.twist.linear.x = self.speed_mps
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        # 3. Publish JointState Topic
        js = JointState()
        js.header.stamp = stamp
        js.name = [
            'front_left_steer_joint',
            'front_right_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        wheel_rot = (self.dist_mm / 1000.0) / 0.033
        js.position = [
            self.steer_angle_rad,
            self.steer_angle_rad,
            wheel_rot,
            wheel_rot,
            wheel_rot,
            wheel_rot
        ]
        self.joint_pub.publish(js)

        # 4. Publish IMU Data Topic (/imu/data)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = qz
        imu.orientation.w = qw
        imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.angular_velocity.z = float(self.cmd_angular_z)
        imu.linear_acceleration.x = float(self.speed_mps)
        self.imu_pub.publish(imu)

    def _try_start_lidar_motor(self):
        if not self.motor_started and self.start_lidar_cli.service_is_ready():
            req = Empty.Request()
            self.start_lidar_cli.call_async(req)
            self.motor_started = True
            if hasattr(self, 'motor_timer') and self.motor_timer:
                self.motor_timer.cancel()
                self.get_logger().info("[LIDAR] Motor start triggered once. Timer cancelled.")

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self._write_serial("S\n")
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SulgHardwareBridge()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
