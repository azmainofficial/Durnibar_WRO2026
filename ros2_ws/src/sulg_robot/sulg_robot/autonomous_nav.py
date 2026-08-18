#!/usr/bin/env python3
"""
autonomous_nav.py – ROS 2 Jazzy Vectorized Optimal Trajectory & Path Planning Node
Subscribes: /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry), /camera/detected_towers
Publishes: /cmd_vel (Twist), /planned_trajectory (Path), /candidate_trajectories (MarkerArray), /wro_state
"""

import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from .optimal_planner import FastAckermannTrajectoryOptimizer

class SulgAutonomousNav(Node):
    def __init__(self):
        super().__init__('sulg_autonomous_nav')

        # Parameters
        self.declare_parameter('challenge_mode', 'OPEN_CHALLENGE')
        self.declare_parameter('nominal_speed_pwm', 120)
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('wheelbase_m', 0.20)
        self.declare_parameter('safety_margin_m', 0.16)

        self.mode = self.get_parameter('challenge_mode').value
        self.nominal_speed_pwm = self.get_parameter('nominal_speed_pwm').value
        self.max_speed_mps = self.get_parameter('max_speed_mps').value
        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.safety_margin = self.get_parameter('safety_margin_m').value

        # Initialize Optimizer
        self.planner = FastAckermannTrajectoryOptimizer(
            wheelbase_m=self.wheelbase,
            safety_margin_m=self.safety_margin
        )

        # State storage
        self.latest_scan = None
        self.latest_odom = None
        self.latest_towers = []
        self.left_wall_m = 0.8
        self.right_wall_m = 0.8
        self.current_yaw_deg = 0.0
        self.start_yaw_deg = None
        self.lap_count = 0
        self.lap_state = "ZONE_1"
        self.enabled = True

        # ROS 2 Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.towers_sub = self.create_subscription(String, '/camera/detected_towers', self.towers_callback, 10)
        self.btn1_sub = self.create_subscription(Bool, '/esp32/button1', self.btn1_callback, 10)

        # ROS 2 Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_trajectory', 10)
        self.candidates_pub = self.create_publisher(MarkerArray, '/candidate_trajectories', 10)
        self.state_pub = self.create_publisher(String, '/wro_state', 10)

        # ROS 2 Services
        self.start_srv = self.create_service(Trigger, '/start_navigation', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_navigation', self.stop_callback)

        # High-frequency 30Hz Planning Loop
        self.timer = self.create_timer(1.0 / 30.0, self.plan_and_control_loop)
        self.get_logger().info(f"[READY] Sulg Autonomous Navigation Node active in {self.mode} mode.")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        # Compute yaw from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = 2.0 * math.atan2(qz, qw)
        self.current_yaw_deg = math.degrees(yaw_rad)

    def towers_callback(self, msg: String):
        try:
            self.latest_towers = json.loads(msg.data)
        except Exception:
            self.latest_towers = []

    def btn1_callback(self, msg: Bool):
        if msg.data:
            self.enabled = not self.enabled
            self.get_logger().info(f"Button 1 pressed: Autonomous navigation {'ENABLED' if self.enabled else 'PAUSED'}")

    def start_callback(self, req, res):
        self.enabled = True
        res.success = True
        res.message = "Autonomous navigation started."
        return res

    def stop_callback(self, req, res):
        self.enabled = False
        self.stop_vehicle()
        res.success = True
        res.message = "Autonomous navigation stopped."
        return res

    def stop_vehicle(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def plan_and_control_loop(self):
        if not self.enabled:
            return

        if self.latest_scan is None:
            return

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = np.array(scan.ranges, dtype=np.float32)

        # Convert scan to (angle_deg, dist_mm) list
        lidar_points = []
        left_dists = []
        right_dists = []

        for i, r in enumerate(ranges):
            if np.isnan(r) or np.isinf(r) or r < 0.05 or r > 4.0:
                continue

            angle_rad = angle_min + i * angle_inc
            angle_deg = (math.degrees(angle_rad) + 360.0) % 360.0
            dist_mm = r * 1000.0
            lidar_points.append((angle_deg, dist_mm))

            # Left wall (60..120 deg)
            if 60.0 <= angle_deg <= 120.0:
                left_dists.append(r)
            # Right wall (240..300 deg)
            elif 240.0 <= angle_deg <= 300.0:
                right_dists.append(r)

        if left_dists:
            self.left_wall_m = 0.7 * self.left_wall_m + 0.3 * float(np.median(left_dists))
        if right_dists:
            self.right_wall_m = 0.7 * self.right_wall_m + 0.3 * float(np.median(right_dists))

        # Format towers in robot coordinates
        fused_towers = []
        for t in self.latest_towers:
            t_dist = t.get('dist_m', 1.0)
            t_angle = t.get('angle_deg', 0.0)
            t_rad = math.radians(t_angle)
            fused_towers.append({
                "color": t.get('color', 'unknown'),
                "dist_m": t_dist,
                "angle_deg": t_angle,
                "x_m": t_dist * math.cos(t_rad),
                "y_m": t_dist * math.sin(t_rad)
            })

        # Run Sub-3ms Vectorized Optimal Trajectory Rollout
        result = self.planner.plan_optimal_trajectory(
            lidar_points=lidar_points,
            towers=fused_towers,
            base_speed_pwm=self.nominal_speed_pwm,
            left_wall_m=self.left_wall_m,
            right_wall_m=self.right_wall_m,
            preferred_angle_deg=0.0,
            challenge_mode=self.mode
        )

        steer_deg = result['steer_deg']
        target_speed_pwm = result['target_speed_pwm']
        target_speed_mps = (target_speed_pwm / 255.0) * self.max_speed_mps
        steer_rad = math.radians(steer_deg)

        # 1. Publish /cmd_vel
        cmd = Twist()
        cmd.linear.x = target_speed_mps
        cmd.angular.z = steer_rad
        self.cmd_vel_pub.publish(cmd)

        # 2. Publish /planned_trajectory (nav_msgs/Path)
        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'base_footprint'

        for pt in result['optimal_path']:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'base_footprint'
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # 3. Publish /candidate_trajectories (visualization_msgs/MarkerArray)
        marker_arr = MarkerArray()
        for idx, cand in enumerate(result['candidate_paths']):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'base_footprint'
            m.ns = 'candidate_rollouts'
            m.id = idx
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.a = 0.4
            if cand['valid']:
                m.color.r = 0.0
                m.color.g = 0.8
                m.color.b = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 0.1
                m.color.b = 0.1

            for p in cand['path']:
                pt_msg = Point()
                pt_msg.x = float(p[0])
                pt_msg.y = float(p[1])
                pt_msg.z = 0.0
                m.points.append(pt_msg)

            marker_arr.markers.append(m)

        self.candidates_pub.publish(marker_arr)

        # 4. Publish /wro_state
        state_msg = String()
        state_info = {
            "mode": self.mode,
            "steer_deg": steer_deg,
            "speed_pwm": target_speed_pwm,
            "latency_ms": result['latency_ms'],
            "cost": result['cost'],
            "safe": result['safe'],
            "left_wall_m": round(self.left_wall_m, 2),
            "right_wall_m": round(self.right_wall_m, 2)
        }
        state_msg.data = json.dumps(state_info)
        self.state_pub.publish(state_msg)

    def destroy_node(self):
        self.stop_vehicle()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SulgAutonomousNav()
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
