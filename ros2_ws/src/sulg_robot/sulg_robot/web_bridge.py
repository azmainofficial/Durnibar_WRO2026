#!/usr/bin/env python3
"""
web_bridge.py – ROS 2 Jazzy Web Dashboard Bridge Node
Serves HTML5 Web Dashboard on Port 5000, streaming ROS 2 topics to browser.
"""

import os
import json
import time
import threading
import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify, request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
import base64
import math

app = Flask(__name__, template_folder=os.path.expanduser('~/pi_code/templates'))

# Shared State
state_lock = threading.Lock()
latest_annotated_frame = None
latest_mask_frame = None
latest_lidar_points = []
latest_optimal_path = []
latest_map_metadata = {
    "resolution": 0.05,
    "width": 0,
    "height": 0,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "origin_yaw": 0.0
}
latest_map_png_base64 = ""

node_instance = None
latest_telemetry = {
    "fps": 30,
    "nearest_color": "NONE",
    "nearest_dist_cm": 0,
    "action": "ROS 2 Active",
    "lidar_left_m": 0.8,
    "lidar_right_m": 0.8,
    "challenge_state": "ROS2_RUNNING",
    "lap_count": 0,
    "esp32_yaw": 0.0,
    "esp32_speed": 0,
    "esp32_x": 0,
    "esp32_y": 0,
    "planner_latency_ms": 1.2,
    "steer_deg": 0.0,
    "steer_pwm": 110,
    "target_speed_pwm": 110,
    "planner_cost": 0.0,
    "planner_safe": True
}

class SulgWebBridge(Node):
    def __init__(self):
        super().__init__('sulg_web_bridge')

        # ROS 2 Subscribers
        self.img_sub = self.create_subscription(Image, '/camera/image_annotated', self.img_callback, 10)
        self.mask_sub = self.create_subscription(Image, '/camera/mask', self.mask_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_trajectory', self.path_callback, 10)
        self.state_sub = self.create_subscription(String, '/wro_state', self.state_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # ROS 2 Publisher for Web Manual Overrides
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info("[READY] ROS 2 Web Bridge active on port 5000.")

    def img_callback(self, msg: Image):
        global latest_annotated_frame
        try:
            # Decode raw byte buffer into numpy array
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            with state_lock:
                latest_annotated_frame = arr
        except Exception:
            pass

    def mask_callback(self, msg: Image):
        global latest_mask_frame
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            with state_lock:
                latest_mask_frame = arr
        except Exception:
            pass

    def scan_callback(self, msg: LaserScan):
        global latest_lidar_points
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.05 < r < 3.5:
                deg = (np.degrees(angle) + 360.0) % 360.0
                points.append([round(float(deg), 1), round(float(r * 1000.0), 1)])
            angle += msg.angle_increment
        with state_lock:
            latest_lidar_points = points

    def odom_callback(self, msg: Odometry):
        global latest_telemetry
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        with state_lock:
            latest_telemetry["esp32_x"] = int(msg.pose.pose.position.x * 1000.0)
            latest_telemetry["esp32_y"] = int(msg.pose.pose.position.y * 1000.0)
            latest_telemetry["esp32_speed"] = int(msg.twist.twist.linear.x * 1000.0)
            latest_telemetry["esp32_yaw"] = round(yaw_deg, 1)

    def path_callback(self, msg: Path):
        global latest_optimal_path
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        with state_lock:
            latest_optimal_path = pts

    def map_callback(self, msg: OccupancyGrid):
        global latest_map_png_base64, latest_map_metadata
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            
            grid = np.frombuffer(msg.data, dtype=np.int8).reshape((h, w))
            
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[grid == 0] = [255, 255, 255]
            img[grid == 100] = [0, 0, 0]
            img[grid < 0] = [150, 150, 150]
            img[(grid > 0) & (grid < 100)] = [100, 100, 100]
            
            img = cv2.flip(img, 0)
            
            ret, png_buf = cv2.imencode('.png', img)
            if ret:
                png_b64 = base64.b64encode(png_buf).decode('utf-8')
                
                q = msg.info.origin.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                
                with state_lock:
                    latest_map_png_base64 = png_b64
                    latest_map_metadata = {
                        "resolution": res,
                        "width": w,
                        "height": h,
                        "origin_x": msg.info.origin.position.x,
                        "origin_y": msg.info.origin.position.y,
                        "origin_yaw": yaw
                    }
        except Exception as e:
            self.get_logger().error(f"Error in map callback: {e}")

    def state_callback(self, msg: String):
        global latest_telemetry
        try:
            data = json.loads(msg.data)
            with state_lock:
                latest_telemetry["challenge_state"] = data.get("mode", "ROS2")
                latest_telemetry["steer_deg"] = data.get("steer_deg", 0.0)
                latest_telemetry["target_speed_pwm"] = data.get("speed_pwm", 110)
                latest_telemetry["planner_latency_ms"] = data.get("latency_ms", 1.0)
                latest_telemetry["planner_cost"] = data.get("cost", 0.0)
                latest_telemetry["planner_safe"] = data.get("safe", True)
                latest_telemetry["lidar_left_m"] = data.get("left_wall_m", 0.8)
                latest_telemetry["lidar_right_m"] = data.get("right_wall_m", 0.8)
        except Exception:
            pass

# Flask Web Server Endpoints
@app.route('/')
def index():
    return render_template('index.html')

def gen_frames(is_mask=False):
    while True:
        with state_lock:
            frame = latest_mask_frame if is_mask else latest_annotated_frame

        if frame is None:
            time.sleep(0.05)
            continue

        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            time.sleep(0.02)
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(False), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mask_feed')
def mask_feed():
    return Response(gen_frames(True), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/telemetry')
def api_telemetry():
    with state_lock:
        t_copy = dict(latest_telemetry)
        t_copy['optimal_path'] = list(latest_optimal_path)
        lidar_copy = list(latest_lidar_points)

    return jsonify({
        "telemetry": t_copy,
        "lidar": lidar_copy,
        "towers": [],
        "config": {
            "green": {"h_min": 35, "h_max": 85, "s_min": 80, "s_max": 255, "v_min": 50, "v_max": 255},
            "red1": {"h_min": 0, "h_max": 10, "s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255},
            "red2": {"h_min": 170, "h_max": 180, "s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255},
            "planner": {
                "w_clearance": 3.5, "w_progress": 2.2, "w_wro_rules": 8.0,
                "w_smooth": 0.8, "safety_margin_m": 0.16, "max_lat_accel": 2.8
            }
        }
    })

@app.route('/api/map')
def api_map():
    with state_lock:
        metadata = dict(latest_map_metadata)
        map_b64 = latest_map_png_base64
    return jsonify({
        "status": "success",
        "metadata": metadata,
        "map_base64": map_b64
    })

@app.route('/api/navigate_to', methods=['POST'])
def api_navigate_to():
    global node_instance
    if node_instance is None:
        return jsonify({"status": "error", "message": "ROS 2 node not initialized"}), 500
        
    payload = request.json or {}
    x = float(payload.get("x", 0.0))
    y = float(payload.get("y", 0.0))
    
    msg = PoseStamped()
    msg.header.stamp = node_instance.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    
    node_instance.goal_pub.publish(msg)
    node_instance.get_logger().info(f"[WEB API] Published navigation goal to Nav2: x={x:.3f}, y={y:.3f}")
    return jsonify({"status": "success", "message": f"Navigation goal published to ({x}, {y})"})

@app.route('/api/cancel_nav', methods=['POST'])
def api_cancel_nav():
    global node_instance
    if node_instance is None:
        return jsonify({"status": "error", "message": "ROS 2 node not initialized"}), 500
    # Stop the robot by sending zero velocity command to /cmd_vel
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    node_instance.cmd_pub.publish(twist)
    node_instance.get_logger().info("[WEB API] Navigation cancelled. Sent zero cmd_vel.")
    return jsonify({"status": "success", "message": "Navigation cancelled."})

def main(args=None):
    global node_instance
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)

    rclpy.init(args=args)
    node_instance = SulgWebBridge()
    node = node_instance

    # Run Flask in background daemon thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True)
    flask_thread.start()

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
