#!/usr/bin/env python3
"""
vision_detector.py – ROS 2 Jazzy Wide-Angle Vision & Color Tower Detection Node
Publishes: /camera/image_raw, /camera/image_annotated, /camera/mask, /camera/detected_towers
"""

import os
import json
import math
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class SulgVisionDetector(Node):
    def __init__(self):
        super().__init__('sulg_vision_detector')

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('fps', 30)
        self.declare_parameter('config_path', os.path.expanduser('~/pi_code/hsv_config.json'))
        self.declare_parameter('calib_path', os.path.expanduser('~/pi_code/camera_calibration.json'))

        self.dev_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.target_fps = self.get_parameter('fps').value
        self.config_path = self.get_parameter('config_path').value
        self.calib_path = self.get_parameter('calib_path').value

        # Publishers
        self.raw_img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.mask_pub = self.create_publisher(Image, '/camera/mask', 10)
        self.towers_pub = self.create_publisher(String, '/camera/detected_towers', 10)

        # Load HSV Config
        self.hsv_config = {
            "green": {"h_min": 35, "h_max": 85, "s_min": 80, "s_max": 255, "v_min": 50, "v_max": 255},
            "red1":  {"h_min": 0, "h_max": 10, "s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255},
            "red2":  {"h_min": 170, "h_max": 180, "s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255}
        }
        self.load_config()

        # Load Camera Calibration
        self.map1, self.map2 = None, None
        self.load_calibration()

        # Initialize Video Capture
        self.cap = cv2.VideoCapture(self.dev_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera on /dev/video{self.dev_id}!")
        else:
            self.get_logger().info(f"[READY] Camera active on /dev/video{self.dev_id} ({self.width}x{self.height} @ {self.target_fps} FPS)")

        # Main processing timer at 30Hz
        self.timer = self.create_timer(1.0 / float(self.target_fps), self.process_frame)

    def load_config(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    cfg = json.load(f)
                    for k in ['green', 'red1', 'red2']:
                        if k in cfg:
                            self.hsv_config[k] = cfg[k]
                self.get_logger().info(f"Loaded HSV parameters from {self.config_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse {self.config_path}: {e}")

    def load_calibration(self):
        if os.path.exists(self.calib_path):
            try:
                with open(self.calib_path, 'r') as f:
                    cal = json.load(f)
                    K = np.array(cal['camera_matrix'], dtype=np.float32)
                    D = np.array(cal['dist_coeffs'], dtype=np.float32)
                    new_K, _ = cv2.getOptimalNewCameraMatrix(K, D, (self.width, self.height), 0)
                    self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (self.width, self.height), cv2.CV_16SC2)
                self.get_logger().info(f"Loaded lens calibration from {self.calib_path}")
            except Exception as e:
                self.get_logger().warn(f"Calibration load failed: {e}")

    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        """Fast conversion from OpenCV numpy array to sensor_msgs/Image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        if encoding == "bgr8":
            msg.step = cv_image.shape[1] * 3
            msg.data = cv_image.tobytes()
        elif encoding == "mono8":
            msg.step = cv_image.shape[1]
            msg.data = cv_image.tobytes()
        return msg

    def process_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        # 1. Undistort if calibrated
        if self.map1 is not None and self.map2 is not None:
            frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()

        # 2. Color segmentation masks
        g_cfg = self.hsv_config['green']
        r1_cfg = self.hsv_config['red1']
        r2_cfg = self.hsv_config['red2']

        mask_g = cv2.inRange(hsv,
            np.array([g_cfg['h_min'], g_cfg['s_min'], g_cfg['v_min']]),
            np.array([g_cfg['h_max'], g_cfg['s_max'], g_cfg['v_max']]))

        mask_r1 = cv2.inRange(hsv,
            np.array([r1_cfg['h_min'], r1_cfg['s_min'], r1_cfg['v_min']]),
            np.array([r1_cfg['h_max'], r1_cfg['s_max'], r1_cfg['v_max']]))

        mask_r2 = cv2.inRange(hsv,
            np.array([r2_cfg['h_min'], r2_cfg['s_min'], r2_cfg['v_min']]),
            np.array([r2_cfg['h_max'], r2_cfg['s_max'], r2_cfg['v_max']]))

        mask_r = cv2.bitwise_or(mask_r1, mask_r2)
        combined_mask = cv2.bitwise_or(mask_g, mask_r)

        detected_towers = []
        cx_img = self.width / 2.0
        fov_h_deg = 109.0

        # Helper contour extractor
        def extract_blobs(mask, color_label, color_bgr):
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2.0
                cy = y + h / 2.0

                # Angle calculation relative to camera centerline
                angle_deg = ((cx - cx_img) / cx_img) * (fov_h_deg / 2.0)
                
                # Approximate distance estimation from bounding box width
                focal_length = 228.0
                known_width_cm = 21.0
                dist_cm = (known_width_cm * focal_length) / max(1.0, float(w))

                detected_towers.append({
                    "color": color_label,
                    "x_px": int(cx),
                    "y_px": int(cy),
                    "width_px": int(w),
                    "height_px": int(h),
                    "angle_deg": round(float(angle_deg), 2),
                    "dist_cm": round(float(dist_cm), 1),
                    "dist_m": round(float(dist_cm) / 100.0, 2)
                })

                # Draw bounding box and label
                cv2.rectangle(annotated, (x, y), (x + w, y + h), color_bgr, 2)
                cv2.putText(annotated, f"{color_label.upper()} {dist_cm:.0f}cm ({angle_deg:+.1f}deg)",
                            (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 2)

        extract_blobs(mask_g, 'green', (0, 255, 0))
        extract_blobs(mask_r, 'red', (0, 0, 255))

        # 3. Publish ROS 2 Images & Tower Data
        self.raw_img_pub.publish(self.cv2_to_imgmsg(frame, "bgr8"))
        self.annotated_pub.publish(self.cv2_to_imgmsg(annotated, "bgr8"))
        self.mask_pub.publish(self.cv2_to_imgmsg(combined_mask, "mono8"))

        tower_msg = String()
        tower_msg.data = json.dumps(detected_towers)
        self.towers_pub.publish(tower_msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SulgVisionDetector()
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
