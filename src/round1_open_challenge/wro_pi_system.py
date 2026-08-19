#!/usr/bin/env python3
"""
wro_pi_system.py – WRO Robotics Vision, RPLiDAR C1 & Web Control Server
Target: Raspberry Pi 5 (16:9 640x360 Video Feed, Live HSV Tuning & LiDAR Radar UI)
"""

import os
import sys
import time
import json
import math
import copy
import serial
import threading
import numpy as np
import cv2
from flask import Flask, render_template, Response, jsonify, request
from disparity_extender import DisparityExtender, LidarCvTowerFusion
from yolo_detector import CameraUndistorter, YoloObjectDetector
from optimal_planner import FastAckermannTrajectoryOptimizer

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hsv_config.json")

def solve_lqr_dare(A, B, Q, R):
    P = np.array(Q, dtype=float)
    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)
    R = float(R)
    
    eps = 1e-4
    for _ in range(100):
        BP = B.T @ P
        denom = R + BP @ B
        if abs(denom[0, 0]) < 1e-6:
            break
        P_next = A.T @ P @ A - (A.T @ P @ B) @ (BP @ A) / denom[0, 0] + Q
        if np.max(np.abs(P_next - P)) < eps:
            P = P_next
            break
        P = P_next
        
    BP = B.T @ P
    K = (BP @ A) / (R + BP @ B)[0, 0]
    return K[0]


# Default Fallback Config (16:9 Aspect Ratio: 640x360)
config_lock = threading.Lock()
config_data = {
    "general": {
        "resize_width": 640,
        "resize_height": 360,
        "dist_threshold_cm": 30.0,
        "known_width_cm": 21.29,
        "min_contour_area": 400,
        "camera_fov_deg": 109.0,
        "focal_length_px": 228.25,
        "bumper_offset_cm": -40.0,
        "target_speed": 55,
        "open_challenge_speed": 55,
        "obstacle_challenge_speed": 55,
        "auto_start_challenge": "IDLE"
    },
    "green": {"h_min": 35, "h_max": 85, "s_min": 80, "s_max": 255, "v_min": 50, "v_max": 255},
    "red1":  {"h_min": 0,  "h_max": 10, "s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255},
    "red2":  {"h_min": 170,"h_max": 180,"s_min": 150, "s_max": 255, "v_min": 100, "v_max": 255},
    "hardware": {
        "lidar_port": "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4d6c14f3473ed11b23e6aeefdf7b791-if00-port0",
        "lidar_baud": 460800,
        "lidar_flip_upside_down": True,
        "lidar_offset_deg": 0,
        "arduino_port": "/dev/serial/by-id/usb-1a86_USB_Single_Serial_56BA018173-if00",
        "arduino_baud": 115200,
        "camera_id": 0
    },
    "control": {"servo_center": 110, "max_adj": 30, "kp": 1.5, "ki": 0.01, "kd": 1.0},
    "planner": {
        "w_clearance": 3.5,
        "w_progress": 2.2,
        "w_wro_rules": 8.0,
        "w_smooth": 0.8,
        "w_centering": 1.5,
        "safety_margin_m": 0.16,
        "horizon_m": 1.8,
        "max_lat_accel": 2.8
    }
}

# ===================== ENGINES (instantiated once) =====================
disp_extender    = DisparityExtender(robot_width_m=0.22, disparity_threshold_m=0.3)
tower_fusion     = LidarCvTowerFusion(cluster_tolerance_m=0.15)
camera_undistorter = CameraUndistorter()
yolo_detector    = YoloObjectDetector(min_area=400)
fast_planner     = FastAckermannTrajectoryOptimizer(
    wheelbase_m=0.20,
    robot_width_m=0.22,
    robot_length_m=0.28,
    max_steer_deg=30.0,
    center_steer_pwm=110,
    num_candidates=21,
    horizon_m=1.8
)

def load_config():
    global config_data
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, 'r') as f:
                cfg = json.load(f)
            with config_lock:
                for key, val in cfg.items():
                    if isinstance(val, dict) and key in config_data:
                        config_data[key].update(val)
                    else:
                        config_data[key] = val
            if "planner" in config_data:
                fast_planner.update_config(config_data["planner"])
            print(f"[CONFIG] Loaded configuration from {CONFIG_PATH}")
        except Exception as e:
            print(f"[CONFIG] Error loading config file: {e}")

def save_config():
    with config_lock:
        snapshot = copy.deepcopy(config_data)
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(snapshot, f, indent=2)
        print(f"[CONFIG] Saved configuration to {CONFIG_PATH}")
        return True
    except Exception as e:
        print(f"[CONFIG] Failed to save config: {e}")
        return False

load_config()

# ===================== GLOBAL SHARED STATE =====================
latest_raw_frame      = None
latest_annotated_frame = None
latest_mask_frame     = None
latest_cv_objects     = []      # [{cx, cy, contour, color}, ...]
latest_lidar_points   = []      # [(angle_deg, dist_mm), ...]
latest_fused_towers   = []

latest_telemetry = {
    "fps": 0.0,
    "nearest_color": "None",
    "nearest_dist_cm": 0.0,
    "action": "No Obstacle",
    "lidar_left_m": 0.0,
    "lidar_right_m": 0.0,
    "lidar_front_m": 0.0,
    "de_target_angle": 0.0,
    "de_target_dist": 0.0,
    "active_mask_color": "green",
    "challenge_state": "IDLE",
    "lap_count": 0,
    "esp32_x": 0.0,
    "esp32_y": 0.0,
    "esp32_yaw": 0.0,
    "esp32_dist": 0.0,
    "esp32_speed": 0.0,
    "optimal_path": [],
    "candidate_paths": [],
    "planner_latency_ms": 0.0,
    "planner_cost": 0.0,
    "steer_deg": 0.0,
    "steer_pwm": 110,
    "target_speed_pwm": 0,
    "planner_safe": True
}
state_lock     = threading.Lock()
active_mask_mode = "green"
challenge_mode = "IDLE"  # IDLE, OPEN_CHALLENGE, OBSTACLE_CHALLENGE, PARKING, MOTOR_TEST
challenge_laps = 0
test_start_time = 0

# Pre-computed blank mask for 16:9 resolution
_BLANK_MASK = np.zeros((360, 640), dtype=np.uint8)

# ===================== VISION PROCESSING =====================
def camera_loop():
    global latest_raw_frame, latest_annotated_frame, latest_mask_frame
    global latest_telemetry, latest_cv_objects

    def _open_camera():
        with config_lock:
            target_id = config_data.get('hardware', {}).get('camera_id', 0)

        try:
            c = cv2.VideoCapture(target_id, cv2.CAP_V4L2)
            if c.isOpened():
                c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
                c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                c.set(cv2.CAP_PROP_FRAME_HEIGHT, 368)
                time.sleep(0.2)
                ret, frame = c.read()
                if ret and frame is not None:
                    print(f"[CAMERA SUCCESS] Fifine K420 online on /dev/video{target_id} (640x368 YUYV)")
                    return c
                c.release()
        except Exception as e:
            print(f"[CAMERA ERROR] {e}")
        return None

    cap = None
    frame_count = 0
    consecutive_drops = 0
    t0 = time.time()

    while True:
        if cap is None or not cap.isOpened():
            cap = _open_camera()
            if cap is None:
                print("[CAMERA] Camera busy/not ready, retrying in 1s...")
                time.sleep(1)
                continue

        ret, raw_frame = cap.read()
        if not ret or raw_frame is None:
            consecutive_drops += 1
            if consecutive_drops >= 5:
                print("[CAMERA WARN] 5 consecutive dropped frames, resetting video device...")
                cap.release()
                cap = None
                consecutive_drops = 0
                time.sleep(1.5)
            else:
                time.sleep(0.02)
            continue

        consecutive_drops = 0

        raw_frame = cv2.resize(raw_frame, (640, 360))

        # Apply Fisheye / Barrel Undistortion
        frame = camera_undistorter.undistort(raw_frame)

        # --- Snapshot config & state (cheap deepcopy only of primitives) ---
        with config_lock:
            cfg = copy.deepcopy(config_data)

        with state_lock:
            current_mask_mode = active_mask_mode
            current_towers    = list(latest_fused_towers)

        # --- Pre-compute optics (avoids redundant trig every detection) ---
        fov_deg      = cfg['general'].get('camera_fov_deg', 109.0)
        W            = cfg['general']['resize_width']
        H            = cfg['general']['resize_height']
        fx           = W / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
        known_w      = cfg['general']['known_width_cm']
        bumper_off   = cfg['general'].get('bumper_offset_cm', -40.0)
        cx_center    = W / 2.0
        half_fov     = fov_deg / 2.0

        # --- Build HSV mask for the preview feed ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if current_mask_mode == 'green':
            cg = cfg['green']
            active_mask = cv2.inRange(
                hsv,
                np.array([cg['h_min'], cg['s_min'], cg['v_min']], np.uint8),
                np.array([cg['h_max'], cg['s_max'], cg['v_max']], np.uint8)
            )
        else:
            cr1, cr2 = cfg['red1'], cfg['red2']
            m1 = cv2.inRange(
                hsv,
                np.array([cr1['h_min'], cr1['s_min'], cr1['v_min']], np.uint8),
                np.array([cr1['h_max'], cr1['s_max'], cr1['v_max']], np.uint8)
            )
            m2 = cv2.inRange(
                hsv,
                np.array([cr2['h_min'], cr2['s_min'], cr2['v_min']], np.uint8),
                np.array([cr2['h_max'], cr2['s_max'], cr2['v_max']], np.uint8)
            )
            active_mask = cv2.bitwise_or(m1, m2)

        # --- Detect bounding boxes via YoloObjectDetector ---
        detections = yolo_detector.detect_bounding_boxes(frame, cfg)

        annotated  = frame.copy()
        nearest    = None
        min_dist   = float('inf')
        cv_objs    = []

        for det in detections:
            x, y, bw, bh = det['bbox']
            width_px   = max(1.0, float(bw))
            color_name = det['class']

            # Camera pinhole distance + bumper offset
            cam_dist_cm = ((known_w * fx) / width_px) + bumper_off

            # LiDAR tower angular lookup
            angle_off    = ((det['cx'] - cx_center) / cx_center) * half_fov
            lidar_dist_cm = None

            for tower in current_towers:
                c_ang = tower['angle_deg']
                if c_ang > 180:
                    c_ang -= 360.0
                if abs(c_ang - angle_off) <= 20.0:
                    lidar_dist_cm = (tower['dist_m'] * 100.0) + bumper_off
                    break

            # Primary: LiDAR; Fallback: camera pinhole
            if lidar_dist_cm is not None:
                dist_cm = lidar_dist_cm
                tag = f"LiDAR {dist_cm:.1f}cm"
            else:
                dist_cm = cam_dist_cm
                tag = f"Cam {dist_cm:.1f}cm"

            if dist_cm < min_dist:
                min_dist = dist_cm
                nearest  = {"color": color_name, "dist": dist_cm,
                            "cx": det['cx'], "cy": det['cy']}

            color_bgr = (0, 255, 0) if color_name == 'green' else (0, 0, 255)
            cv2.rectangle(annotated, (x, y), (x + bw, y + bh), color_bgr, 2)
            cv2.circle(annotated, (det['cx'], det['cy']), 5, (255, 255, 255), -1)
            cv2.putText(annotated,
                        f"[{color_name}] {tag} ({bw}px)",
                        (x, max(12, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv_objs.append({
                "cx": det['cx'], "cy": det['cy'],
                "contour": det['contour'], "color": color_name
            })

        # --- Decision ---
        action_text = "No Obstacle"
        if nearest and nearest['dist'] <= cfg['general']['dist_threshold_cm']:
            action_text = ("HARD RIGHT (Green Block)"
                           if nearest['color'] == 'green'
                           else "HARD LEFT (Red Block)")

        # --- HUD Overlays (Y coords clamped to 16:9 frame height) ---
        cv2.putText(annotated,
                    f"WRO 16:9 640x360 | Mode: {current_mask_mode.upper()}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(annotated,
                    f"Action: {action_text}",
                    (10, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if nearest is None else (0, 0, 255), 2)

        # --- FPS counter ---
        frame_count += 1
        t_now = time.time()
        fps   = frame_count / (t_now - t0 + 1e-9)
        if frame_count >= 30:
            frame_count = 0
            t0 = t_now

        # --- Publish to shared state (single lock section) ---
        with state_lock:
            latest_raw_frame       = raw_frame          # unmodified for color picking
            latest_annotated_frame = annotated
            latest_mask_frame      = active_mask        # already 16:9 shape (360,640)
            latest_cv_objects      = cv_objs            # FIX: was all_objs (NameError)
            latest_telemetry['fps']              = round(fps, 1)
            latest_telemetry['nearest_color']    = nearest['color'] if nearest else "None"
            latest_telemetry['nearest_dist_cm']  = round(nearest['dist'], 1) if nearest else 0.0
            latest_telemetry['action']           = action_text
            latest_telemetry['active_mask_color']= current_mask_mode

# ===================== RPLIDAR C1 THREAD =====================
def lidar_loop():
    global latest_lidar_points, latest_telemetry

    with config_lock:
        port      = config_data['hardware']['lidar_port']
        baud      = config_data['hardware']['lidar_baud']

    print(f"[LIDAR] Connecting RPLiDAR C1 on {port} @ {baud} baud...")

    while True:
        try:
            if not os.path.exists(port):
                time.sleep(2)
                continue

            ser = serial.Serial(port, baud, timeout=1)
            ser.dtr = False  # Turn on RPLiDAR C1 motor & laser diode
            time.sleep(0.2)
            ser.reset_input_buffer()

            # Reset RPLiDAR state machine with STOP command
            ser.write(b'\xa5\x25')
            time.sleep(0.15)
            ser.reset_input_buffer()

            # GET_INFO check
            ser.write(b'\xa5\x50')
            time.sleep(0.15)
            resp = ser.read(27)

            if len(resp) < 27 or not resp.startswith(b'\xa5\x5a'):
                print(f"[LIDAR WARN] RPLiDAR GET_INFO failed (len={len(resp)}), retrying...")
                ser.close()
                time.sleep(1.5)
                continue

            print("[LIDAR SUCCESS] RPLiDAR C1 Ready & Motor Active!")

            # Standard Scan command
            ser.reset_input_buffer()
            ser.write(b'\xa5\x20')
            time.sleep(0.15)
            desc = ser.read(7)
            if len(desc) < 7 or not desc.startswith(b'\xa5\x5a'):
                print("[LIDAR WARN] RPLiDAR SCAN descriptor failed, retrying...")
                ser.close()
                time.sleep(1.5)
                continue

            points = []
            while True:
                # Read 1 byte to check start bit framing alignment
                b0 = ser.read(1)
                if len(b0) < 1:
                    continue

                # Byte 0 Bit 0 (S) and Bit 1 (!S) must be different
                s = b0[0] & 0x01
                not_s = (b0[0] >> 1) & 0x01
                if s == not_s:
                    # Misaligned! Skip this byte to resynchronize framing
                    continue

                raw_data = ser.read(4)
                if len(raw_data) < 4:
                    continue

                quality   = b0[0] >> 2
                angle_q6  = ((raw_data[0] >> 1) | (raw_data[1] << 7))
                angle_deg = angle_q6 / 64.0
                dist_q2   = (raw_data[2] | (raw_data[3] << 8))
                dist_mm   = dist_q2 / 4.0

                with config_lock:
                    hw = config_data.get("hardware", {})
                    flip_lidar = hw.get("lidar_flip_upside_down", True)
                    offset_deg = hw.get("lidar_offset_deg", 0)
                    min_deg = hw.get("lidar_scan_min_deg", 0)
                    max_deg = hw.get("lidar_scan_max_deg", 360)
                    max_dist_m = hw.get("lidar_max_range_m", 3.0)
                    target_freq_hz = hw.get("lidar_scan_freq_hz", 15)

                if flip_lidar:
                    angle_deg = (360.0 - angle_deg) % 360.0
                angle_deg = (angle_deg + offset_deg) % 360.0

                # Check angle sector bounds
                if min_deg <= max_deg:
                    in_sector = (min_deg <= angle_deg <= max_deg)
                else:
                    in_sector = (angle_deg >= min_deg or angle_deg <= max_deg)

                if in_sector and quality > 0 and 50 < dist_mm <= (max_dist_m * 1000.0):
                    points.append((round(angle_deg, 1), round(dist_mm, 1)))

                # Dynamic scan batch size based on scan frequency
                pts_threshold = max(90, min(720, int(360.0 * (15.0 / max(1, target_freq_hz)))))
                if len(points) >= pts_threshold:
                    # Wall distances
                    left_d  = [d for a, d in points if  60 <= a <=  90]
                    right_d = [d for a, d in points if 270 <= a <= 300]
                    front_d = [d for a, d in points if a >= 340 or a <= 20]

                    avg_left  = round((sum(left_d)  / len(left_d))  / 1000.0, 2) if left_d  else 0.0
                    avg_right = round((sum(right_d) / len(right_d)) / 1000.0, 2) if right_d else 0.0
                    avg_front = round((sum(front_d) / len(front_d)) / 1000.0, 2) if front_d else 0.0

                    # Disparity Extender & Tower Clustering
                    de_target_a, de_target_d, _ = disp_extender.process_scan(points)
                    raw_clusters = tower_fusion.cluster_lidar_points(points)

                    with config_lock:
                        fov_deg = config_data.get('general', {}).get('camera_fov_deg', 109.0)

                    with state_lock:
                        cv_objs = list(latest_cv_objects)
                        current_c_mode = challenge_mode

                    fused_towers = tower_fusion.fuse_towers(
                        raw_clusters, cv_objs,
                        fov_deg=fov_deg, image_width=640
                    )

                    # Compute real-time optimal trajectory for live visualization and planning
                    plan_res = fast_planner.plan_optimal_trajectory(
                        lidar_points=points,
                        towers=fused_towers,
                        base_speed_pwm=110,
                        left_wall_m=avg_left,
                        right_wall_m=avg_right,
                        preferred_angle_deg=de_target_a,
                        challenge_mode=current_c_mode
                    )

                    with state_lock:
                        latest_lidar_points          = list(points)
                        latest_fused_towers          = fused_towers
                        latest_telemetry['lidar_left_m']     = avg_left
                        latest_telemetry['lidar_right_m']    = avg_right
                        latest_telemetry['lidar_front_m']    = avg_front
                        latest_telemetry['de_target_angle']  = de_target_a
                        latest_telemetry['de_target_dist']   = round(de_target_d, 2)
                        latest_telemetry['lidar_scan_freq_hz'] = target_freq_hz
                        latest_telemetry['optimal_path']     = plan_res['optimal_path']
                        latest_telemetry['candidate_paths']   = plan_res['candidate_paths']
                        latest_telemetry['planner_latency_ms'] = plan_res['latency_ms']
                        latest_telemetry['planner_cost']     = plan_res['cost']
                        latest_telemetry['planner_safe']     = plan_res['safe']
                        if challenge_mode == "IDLE":
                            latest_telemetry['steer_deg']       = plan_res['steer_deg']
                            latest_telemetry['steer_pwm']       = plan_res['steer_pwm']
                            latest_telemetry['target_speed_pwm'] = 0

                    points = []

        except Exception as e:
            print(f"[LIDAR ERROR] {e}")
            time.sleep(2)

# ===================== FLASK WEB SERVER =====================
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

def gen_mjpeg_frames(get_frame_func):
    """MJPEG stream generator – reads frame atomically then encodes outside the lock."""
    while True:
        with state_lock:
            frame = get_frame_func()
            frame_copy = frame.copy() if frame is not None else None

        if frame_copy is None:
            time.sleep(0.03)
            continue

        # Encode JPEG outside the lock to minimise contention
        ret, jpeg = cv2.imencode('.jpg', frame_copy,
                                 [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.033)  # ~30 FPS cap

def _mask_to_bgr(mask):
    """Convert a grayscale mask to 3-channel BGR so it can be JPEG-encoded."""
    if mask is None:
        return None
    if len(mask.shape) == 2:
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    return mask

@app.route('/video_feed')
def video_feed():
    return Response(
        gen_mjpeg_frames(lambda: latest_annotated_frame),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/mask_feed')
def mask_feed():
    def get_mask_bgr():
        return _mask_to_bgr(latest_mask_frame)
    return Response(
        gen_mjpeg_frames(get_mask_bgr),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/api/calibrate_distance', methods=['POST'])
def api_calibrate_distance():
    payload   = request.json or {}
    known_cm  = float(payload.get("known_dist_cm", 30.0))

    with state_lock:
        cv_objs  = list(latest_cv_objects)
        raw_frame = latest_raw_frame.copy() if latest_raw_frame is not None else None

    if not cv_objs or raw_frame is None:
        return jsonify({"status": "error", "message": "No obstacle detected in camera view"}), 400

    largest_obj = max(cv_objs, key=lambda o: cv2.contourArea(o['contour']))
    _, _, bw, _ = cv2.boundingRect(largest_obj['contour'])
    width_px    = float(max(1, bw))

    with config_lock:
        fov_deg = config_data['general'].get('camera_fov_deg', 109.0)
        w0      = config_data['general'].get('resize_width', 640)
        fx      = w0 / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
        cal_w   = round((known_cm * width_px) / fx, 2)
        config_data['general']['known_width_cm'] = cal_w

    save_config()
    print(f"[CALIBRATE] width_px={width_px} at {known_cm}cm → known_width_cm={cal_w}")
    return jsonify({"status": "success", "calibrated_width_cm": cal_w, "width_px": width_px})

@app.route('/api/calibrate_dual_targets', methods=['POST'])
def api_calibrate_dual_targets():
    payload        = request.json or {}
    green_target   = float(payload.get("green_dist_cm", 41.0))
    red_target     = float(payload.get("red_dist_cm",   95.0))

    with state_lock:
        cv_objs       = list(latest_cv_objects)
        current_towers = list(latest_fused_towers)

    with config_lock:
        fov_deg = config_data['general'].get('camera_fov_deg', 109.0)
        w0      = config_data['general'].get('resize_width', 640)
        fx      = w0 / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
        cx_c    = w0 / 2.0
        half_fov = fov_deg / 2.0

    results = {}

    def _get_tower_for_obj(obj):
        ang = ((obj['cx'] - cx_c) / cx_c) * half_fov
        return next(
            (t for t in current_towers
             if abs((t['angle_deg'] if t['angle_deg'] <= 180 else t['angle_deg'] - 360) - ang) <= 20.0),
            None
        )

    green_obj = next((o for o in cv_objs if o['color'] == 'green'), None)
    red_obj   = next((o for o in cv_objs if o['color'] == 'red'),   None)

    offsets = []
    for obj, target_cm, label in [(green_obj, green_target, 'green'),
                                   (red_obj,   red_target,   'red')]:
        if obj is None:
            continue
        tower = _get_tower_for_obj(obj)
        if tower:
            b_off = round(target_cm - tower['dist_m'] * 100.0, 1)
            results[f'{label}_bumper_offset_cm'] = b_off
            offsets.append(b_off)
        else:
            _, _, bw, _ = cv2.boundingRect(obj['contour'])
            if bw > 0:
                cal_w = round((target_cm * bw) / fx, 2)
                results[f'{label}_cam_width_cm'] = cal_w
                with config_lock:
                    config_data['general']['known_width_cm'] = cal_w

    # Average bumper offset from both targets
    if offsets:
        avg_offset = round(sum(offsets) / len(offsets), 1)
        with config_lock:
            config_data['general']['bumper_offset_cm'] = avg_offset
        results['bumper_offset_cm_applied'] = avg_offset

    save_config()
    with config_lock:
        general = dict(config_data['general'])
    return jsonify({"status": "success", "results": results, "config": general})

@app.route('/api/toggle_undistort', methods=['POST'])
def api_toggle_undistort():
    payload = request.json or {}
    camera_undistorter.enabled = bool(payload.get("enabled", True))
    return jsonify({"status": "success", "undistort_enabled": camera_undistorter.enabled})

@app.route('/api/run_checkerboard_calib', methods=['POST'])
def api_run_checkerboard_calib():
    import subprocess
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibrate_camera.py")
    subprocess.Popen(["python3", script, "9", "6"])
    return jsonify({"status": "success",
                    "message": "Checkerboard calibration started in background."})

@app.route('/api/telemetry')
def api_telemetry():
    with state_lock:
        data = {
            "telemetry": dict(latest_telemetry),
            "lidar":     list(latest_lidar_points),
            "towers":    list(latest_fused_towers),
        }
    with config_lock:
        data["config"] = copy.deepcopy(config_data)
    return jsonify(data)

@app.route('/api/set_hsv', methods=['POST'])
def api_set_hsv():
    payload = request.json
    if not payload:
        return jsonify({"status": "error", "message": "No JSON payload"}), 400

    color = payload.get("color", "green")
    with config_lock:
        if color in config_data:
            config_data[color].update(payload.get("values", {}))
        if "general" in payload:
            config_data["general"].update(payload["general"])
        if "hardware" in payload:
            config_data["hardware"].update(payload["hardware"])

    save_config()
    with config_lock:
        cfg_out = copy.deepcopy(config_data)
    return jsonify({"status": "success", "config": cfg_out})

@app.route('/api/set_planner_config', methods=['POST'])
def api_set_planner_config():
    payload = request.json
    if not payload:
        return jsonify({"status": "error", "message": "No JSON payload"}), 400

    with config_lock:
        if "planner" not in config_data:
            config_data["planner"] = {}
        config_data["planner"].update(payload)
        fast_planner.update_config(config_data["planner"])
        if "general" in payload:
            config_data["general"].update(payload["general"])

    save_config()
    with config_lock:
        cfg_out = copy.deepcopy(config_data)
    return jsonify({"status": "success", "config": cfg_out})

@app.route('/api/set_mask_mode', methods=['POST'])
def api_set_mask_mode():
    global active_mask_mode
    payload = request.json or {}
    mode    = payload.get("mode", "green")
    with state_lock:
        active_mask_mode = mode
    return jsonify({"status": "success", "mode": active_mask_mode})

@app.route('/api/pick_color', methods=['POST'])
def api_pick_color():
    payload      = request.json or {}
    px           = payload.get("x", 320)
    py           = payload.get("y", 180)
    target_color = payload.get("color", "green")

    with state_lock:
        frame = latest_raw_frame.copy() if latest_raw_frame is not None else None

    if frame is None:
        return jsonify({"status": "error", "message": "No frame available"}), 400

    h, w = frame.shape[:2]
    px = max(0, min(w - 1, int(px)))
    py = max(0, min(h - 1, int(py)))

    x1, x2 = max(0, px - 2), min(w, px + 3)
    y1, y2 = max(0, py - 2), min(h, py + 3)
    patch   = frame[y1:y2, x1:x2]

    hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    avg_h = int(np.median(hsv_patch[:, :, 0]))
    avg_s = int(np.median(hsv_patch[:, :, 1]))
    avg_v = int(np.median(hsv_patch[:, :, 2]))

    new_hsv = {
        "h_min": max(0,   avg_h - 15),
        "h_max": min(180, avg_h + 15),
        "s_min": max(30,  avg_s - 50),
        "s_max": 255,
        "v_min": max(30,  avg_v - 50),
        "v_max": 255,
    }

    with config_lock:
        if target_color in config_data:
            config_data[target_color].update(new_hsv)

    save_config()
    print(f"[COLOR PICK] ({px},{py}) → HSV({avg_h},{avg_s},{avg_v}) → {target_color}: {new_hsv}")
    return jsonify({"status": "success",
                    "sampled_hsv": [avg_h, avg_s, avg_v],
                    "new_hsv": new_hsv})

@app.route('/api/start_challenge', methods=['POST'])
def api_start_challenge():
    global challenge_mode
    payload = request.json or {}
    mode = payload.get("challenge", "OPEN_CHALLENGE")
    if mode in ["OPEN_CHALLENGE", "OBSTACLE_CHALLENGE", "PARKING", "IDLE"]:
        challenge_mode = mode
        print(f"[API] Set challenge mode to {challenge_mode}")
        return jsonify({"status": "success", "challenge_mode": challenge_mode})
    return jsonify({"status": "error", "message": "Invalid challenge mode"}), 400

@app.route('/api/stop_challenge', methods=['POST'])
def api_stop_challenge():
    global challenge_mode
    challenge_mode = "IDLE"
    print("[API] Challenge stopped manually.")
    return jsonify({"status": "success", "challenge_mode": challenge_mode})

@app.route('/api/test_motor', methods=['POST'])
def api_test_motor():
    global challenge_mode, test_start_time
    challenge_mode = "MOTOR_TEST"
    test_start_time = time.time()
    print("[API] Triggered 2-second motor diagnostic test.")
    return jsonify({"status": "success", "message": "Motor test triggered"})

# ===================== ESP32 SERIAL MOTION CONTROLLER LOOP =====================
def esp32_loop():
    global latest_telemetry, challenge_mode, challenge_laps, test_start_time
    
    with config_lock:
        port = config_data.get("hardware", {}).get("arduino_port", "/dev/ttyUSB1")
        baud = config_data.get("hardware", {}).get("arduino_baud", 115200)
        
    print(f"[ESP32] Connecting ESP32 on {port} @ {baud} baud...")
    
    lap_count = 0
    cumulative_yaw = 0.0
    last_yaw = None
    start_dist = 0
    final_move_start_dist = None
    last_btn1 = False
    last_btn2 = False
    
    prev_challenge_mode = "IDLE"
    auto_boot_checked = False
    ready_indicator_sent = False
    connection_time = time.time()
    
    center_steer = 110
    warning_dist_cm = 120.0
    safety_angle_bias = 25.0
    
    error_integral = 0.0
    last_error = 0.0
    last_pid_time = time.time()
    
    parking_step = 0
    parking_start_time = 0
    
    stuck_recovery_mode = False
    stuck_start_time = 0.0
    stuck_reverse_dir = 110
    
    while True:
        try:
            if not os.path.exists(port):
                time.sleep(2)
                continue
                
            ser = serial.Serial(port, baud, timeout=0.1)
            ser.reset_input_buffer()
            print("[ESP32 SUCCESS] Connected to ESP32 Motion Controller!")
            connection_time = time.time()
            
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("ODOM,"):
                    continue
                    
                parts = line.split(',')
                if len(parts) < 9:
                    continue
                    
                esp_x = int(parts[1])
                esp_y = int(parts[2])
                esp_yaw = float(parts[3])
                esp_dist = int(parts[4])
                esp_speed = int(parts[5])
                esp_sensors_ok = bool(int(parts[6]))
                esp_btn1 = bool(int(parts[7]))
                esp_btn2 = bool(int(parts[8]))
                
                # ── Ready State Indicator: Glow Yellow LED when system is fully ready to run ──
                with state_lock:
                    has_lidar = len(latest_lidar_points) > 0
                    has_cam = latest_telemetry.get('fps', 0.0) > 0.0

                if challenge_mode == "IDLE" and has_lidar and not ready_indicator_sent:
                    ser.write(b"S\nY 1\nG 0\nL 0\n")
                    ready_indicator_sent = True
                    print("[SYSTEM READY] >>> RPLiDAR + Camera Active! Yellow LED GLOWING. Standing still. Press Button 1 to Run! <<<")

                if last_yaw is not None:
                    dyaw = esp_yaw - last_yaw
                    if dyaw > 180.0:
                        dyaw -= 360.0
                    elif dyaw < -180.0:
                        dyaw += 360.0
                    cumulative_yaw += dyaw
                last_yaw = esp_yaw
                
                btn1_pressed = esp_btn1 and not last_btn1
                btn2_pressed = esp_btn2 and not last_btn2
                last_btn1 = esp_btn1
                last_btn2 = esp_btn2

                # BUTTON 1: RUN THE BOT (Starts Open Challenge 3 Laps)
                if btn1_pressed:
                    challenge_mode = "OPEN_CHALLENGE"
                    lap_count = 0
                    cumulative_yaw = 0.0
                    start_dist = esp_dist
                    stuck_recovery_mode = False
                    ser.write(b"Y 0\nG 1\nL 0\nB\n")
                    print("[HARDWARE BUTTON 1] Pressed -> RUNNING BOT (Open Challenge, 3 Laps, Speed 55)!")
                        
                # BUTTON 2: STOP THE BOT (Emergency Stop / IDLE)
                if btn2_pressed:
                    challenge_mode = "IDLE"
                    stuck_recovery_mode = False
                    ser.write(b"S\nY 1\nG 0\nL 0\nB\n")
                    print("[HARDWARE BUTTON 2] Pressed -> EMERGENCY STOP (Bot Stopped / IDLE)!")

                # ── Global State Transition Handler (Ensures clean start from Web UI, Button, or Auto-Boot) ──
                if challenge_mode != prev_challenge_mode:
                    print(f"[STATE CHANGE] {prev_challenge_mode} -> {challenge_mode}")
                    if challenge_mode in ["OPEN_CHALLENGE", "OBSTACLE_CHALLENGE"]:
                        lap_count = 0
                        cumulative_yaw = 0.0
                        start_dist = esp_dist
                        last_yaw = esp_yaw
                        final_move_start_dist = None
                        ser.write(b"R\nY 0\nG 1\nL 0\n") # Reset ESP32 odom, Yellow OFF, Green ON
                        ready_indicator_sent = False
                        time.sleep(0.03)
                    elif challenge_mode == "IDLE":
                        ser.write(b"S\nY 1\nG 0\nL 0\n") # Stop motor, Yellow ON (Ready), Green OFF
                        ready_indicator_sent = True
                    prev_challenge_mode = challenge_mode
                
                with config_lock:
                    target_speed = config_data.get("general", {}).get("target_speed", 55)
                    open_speed = config_data.get("general", {}).get("open_challenge_speed", target_speed)
                    obstacle_speed = config_data.get("general", {}).get("obstacle_challenge_speed", target_speed)

                if challenge_mode in ["OPEN_CHALLENGE", "OBSTACLE_CHALLENGE"]:
                    calculated_laps = int(abs(cumulative_yaw) / 355.0)
                    if calculated_laps > lap_count:
                        lap_count = calculated_laps
                        print(f"[CHALLENGE] Completed Lap {lap_count} / 3! (Cumulative Yaw: {cumulative_yaw:.1f}°)")
                        
                # STATE 1: OPEN CHALLENGE (3 Laps maximum speed 55, strict wall clearance & stuck recovery)
                if challenge_mode == "OPEN_CHALLENGE":
                    if lap_count >= 3:
                        if final_move_start_dist is None:
                            final_move_start_dist = esp_dist
                            print(f"[CHALLENGE] 3 laps complete! Starting final 15cm forward move. start_dist={final_move_start_dist}")
                        
                        dist_traveled = abs(esp_dist - final_move_start_dist)
                        if dist_traveled >= 150: # 150mm = 15cm
                            challenge_mode = "IDLE"
                            ser.write(b"S\nY 1\nG 0\nL 0\nB\n")
                            print(f"[CHALLENGE SUCCESS] Finished 15cm forward move (traveled {dist_traveled}mm). Stopping bot.")
                            final_move_start_dist = None
                        else:
                            # Drive forward straight to clear the line
                            cmd = "D 45 110\n"
                            ser.write(cmd.encode('utf-8'))
                        continue
                        
                    with state_lock:
                        left_dist = latest_telemetry.get('lidar_left_m', 0.8)
                        right_dist = latest_telemetry.get('lidar_right_m', 0.8)
                        front_dist = latest_telemetry.get('lidar_front_m', 1.5)
                        de_ang = latest_telemetry.get('de_target_angle', 0.0)

                    center_pwm = 110
                    base_spd = int(open_speed)

                    # ── FRONT WALL STUCK RECOVERY ("BACK STEP") ──
                    if stuck_recovery_mode:
                        dt_stuck = time.time() - stuck_start_time
                        if dt_stuck < 0.9:
                            # Step 1: Reverse back while pointing wheels opposite
                            cmd = f"D -58 {stuck_reverse_dir}\n"
                            ser.write(cmd.encode('utf-8'))
                            action_name = "STUCK_REVERSE_BACKSTEP"
                            steer = stuck_reverse_dir
                            speed = -58
                        elif dt_stuck < 1.4:
                            # Step 2: Forward pivot into the open corridor
                            fwd_steer = 50 if (stuck_reverse_dir > 110) else 170
                            cmd = f"D 42 {fwd_steer}\n"
                            ser.write(cmd.encode('utf-8'))
                            action_name = "STUCK_REALIGN_FORWARD"
                            steer = fwd_steer
                            speed = 42
                        else:
                            stuck_recovery_mode = False
                            print("[RECOVERY COMPLETE] Back step done! Resuming Open Challenge navigation.")
                            
                        with state_lock:
                            latest_telemetry['steer_pwm']       = steer
                            latest_telemetry['target_speed_pwm'] = speed
                            latest_telemetry['action']          = action_name
                        continue

                    # Trigger recovery if front wall is closer than 0.28m (stuck in front of wall)
                    if 0.02 < front_dist < 0.28:
                        stuck_recovery_mode = True
                        stuck_start_time = time.time()
                        # If more room on left, reverse wheels RIGHT (165) so nose swings LEFT towards open room
                        # If more room on right, reverse wheels LEFT (55) so nose swings RIGHT towards open room
                        if left_dist >= right_dist:
                            stuck_reverse_dir = 165
                        else:
                            stuck_reverse_dir = 55
                        print(f"[FRONT WALL STUCK] Front obstacle at {front_dist:.2f}m! Initiating Back Step reverse...")
                        cmd = f"D -58 {stuck_reverse_dir}\n"
                        ser.write(cmd.encode('utf-8'))
                        continue

                    # ── NORMAL OPEN CHALLENGE NAVIGATION (CORRECTED STEERING POLARITY) ──
                    # 1. Critical Side Wall Proximity Recovery (< 0.28m)
                    if 0.05 < left_dist < 0.28:
                        # Too close to left wall -> steer RIGHT (PWM > 110) away from it
                        steer = 160
                        speed = max(38, base_spd - 10)
                        action_name = "EMERGENCY_RIGHT"
                    elif 0.05 < right_dist < 0.28:
                        # Too close to right wall -> steer LEFT (PWM < 110) away from it
                        steer = 60
                        speed = max(38, base_spd - 10)
                        action_name = "EMERGENCY_LEFT"
                    # 2. Corner Navigation (Front Wall within 0.85m)
                    elif 0.05 < front_dist < 0.85:
                        urgency = max(0.0, min(1.0, (0.85 - front_dist) / 0.55))
                        speed = max(40, int(round(base_spd * (1.0 - 0.25 * urgency))))
                        if left_dist >= right_dist:
                            # More room on left -> Turn LEFT (PWM < 110)
                            steer = max(50, int(round(center_pwm - 60.0 * (0.65 + 0.35 * urgency))))
                            action_name = "CORNER_LEFT"
                        else:
                            # More room on right -> Turn RIGHT (PWM > 110)
                            steer = min(170, int(round(center_pwm + 60.0 * (0.65 + 0.35 * urgency))))
                            action_name = "CORNER_RIGHT"
                    # 3. Straight Track Centerline Tracking & Gap Bias
                    else:
                        err = left_dist - right_dist
                        # If left_dist < right_dist (closer to left), err < 0 -> steer right (PWM > 110)
                        # If right_dist < left_dist (closer to right), err > 0 -> steer left (PWM < 110)
                        steer_adj = -int(round(35.0 * max(-1.0, min(1.0, err))))
                        # Factor in gap angle if available
                        gap_adj = -int(round(de_ang * 0.4))
                        total_adj = max(-50, min(50, steer_adj + gap_adj))
                        steer = max(60, min(160, center_pwm + total_adj))
                        speed = base_spd
                        action_name = "STRAIGHT_CENTERING"

                    cmd = f"D {speed} {steer}\n"
                    ser.write(cmd.encode('utf-8'))

                    with state_lock:
                        latest_telemetry['steer_pwm']       = steer
                        latest_telemetry['steer_deg']       = round((steer - 110) * (30.0 / 60.0), 1)
                        latest_telemetry['target_speed_pwm'] = speed
                        latest_telemetry['action']          = action_name
                        latest_telemetry['planner_safe']     = True
                        
                # STATE 2: OBSTACLE CHALLENGE (Evasion of Green & Red pillars + trajectory optimization)
                elif challenge_mode == "OBSTACLE_CHALLENGE":
                    if lap_count >= 3 and (esp_dist - start_dist) >= 22000:
                        challenge_mode = "PARKING"
                        parking_step = 0
                        parking_start_time = time.time()
                        print("[CHALLENGE] Completed 3 laps! Commencing Parallel Parking.")
                        continue
                        
                    with state_lock:
                        pts = list(latest_lidar_points)
                        towers = list(latest_fused_towers)
                        de_ang = latest_telemetry.get('de_target_angle', 0.0)
                        left_dist = latest_telemetry.get('lidar_left_m', 0.8)
                        right_dist = latest_telemetry.get('lidar_right_m', 0.8)

                    # Compute optimal trajectory avoiding red/green obstacles with rule adherence
                    plan_res = fast_planner.plan_optimal_trajectory(
                        lidar_points=pts,
                        towers=towers,
                        current_speed_pwm=esp_speed,
                        base_speed_pwm=obstacle_speed,
                        left_wall_m=left_dist,
                        right_wall_m=right_dist,
                        preferred_angle_deg=de_ang,
                        challenge_mode="OBSTACLE_CHALLENGE"
                    )

                    steer = plan_res['steer_pwm']
                    speed = plan_res['target_speed_pwm']
                    cmd = f"D {speed} {steer}\n"
                    ser.write(cmd.encode('utf-8'))

                    with state_lock:
                        latest_telemetry['optimal_path']     = plan_res['optimal_path']
                        latest_telemetry['candidate_paths']   = plan_res['candidate_paths']
                        latest_telemetry['planner_latency_ms'] = plan_res['latency_ms']
                        latest_telemetry['planner_cost']     = plan_res['cost']
                        latest_telemetry['steer_deg']       = plan_res['steer_deg']
                        latest_telemetry['steer_pwm']       = steer
                        latest_telemetry['target_speed_pwm'] = speed
                        latest_telemetry['planner_safe']     = plan_res['safe']
                    
                # STATE 3: PARALLEL PARKING
                elif challenge_mode == "PARKING":
                    now = time.time()
                    dt = now - parking_start_time
                    
                    if parking_step == 0:
                        ser.write(b"S\n")
                        parking_step = 1
                        parking_start_time = now
                    elif parking_step == 1:
                        if dt > 0.8:
                            ser.write(b"D -65 50\n")
                            parking_step = 2
                            parking_start_time = now
                    elif parking_step == 2:
                        if dt > 1.8:
                            ser.write(b"D -65 170\n")
                            parking_step = 3
                            parking_start_time = now
                    elif parking_step == 3:
                        if dt > 1.6:
                            ser.write(b"S\n")
                            challenge_mode = "IDLE"
                            parking_step = 0
                            print("[CHALLENGE] Parallel parking complete!")
                            
                # STATE 4: MOTOR TEST DIAGNOSTIC RUN
                elif challenge_mode == "MOTOR_TEST":
                    now = time.time()
                    dt = now - test_start_time
                    if dt < 2.0:
                        cmd = f"D {target_speed} 110\n"
                        ser.write(cmd.encode('utf-8'))
                    else:
                        ser.write(b"S\n")
                        challenge_mode = "IDLE"
                        print("[CHALLENGE] Motor diagnostic test complete!")
                            
                with state_lock:
                    challenge_laps = lap_count
                    latest_telemetry['challenge_state'] = challenge_mode
                    latest_telemetry['lap_count'] = lap_count
                    latest_telemetry['esp32_x'] = esp_x
                    latest_telemetry['esp32_y'] = esp_y
                    latest_telemetry['esp32_yaw'] = esp_yaw
                    latest_telemetry['esp32_dist'] = esp_dist
                    latest_telemetry['esp32_speed'] = esp_speed
                    
        except Exception as e:
            print(f"[ESP32 ERROR] connection lost: {e}")
            time.sleep(2)

def main():
    t_cam   = threading.Thread(target=camera_loop, daemon=True, name="CameraThread")
    t_lidar = threading.Thread(target=lidar_loop,  daemon=True, name="LidarThread")
    t_esp   = threading.Thread(target=esp32_loop,  daemon=True, name="Esp32Thread")
    t_cam.start()
    t_lidar.start()
    t_esp.start()

    print("=" * 60)
    print("  WRO Autonomous Navigation Engine Running (Headless Competition Mode)")
    print("  Web dashboard is DISABLED for maximum standalone performance.")
    print("  - Button 1: Start Open Challenge (3 Laps @ Speed 55)")
    print("  - Button 2: Emergency Stop")
    print("=" * 60)

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("[SHUTDOWN] Exiting WRO navigation engine.")

if __name__ == '__main__':
    main()
