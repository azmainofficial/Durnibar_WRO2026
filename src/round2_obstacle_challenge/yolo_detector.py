#!/usr/bin/env python3
"""
yolo_detector.py – Bounding Box (x, y, w, h) Object Detector & Undistortion Engine
For 16:9 Widescreen USB Wide-Angle Camera on Raspberry Pi 5
"""

import os
import json
import numpy as np
import cv2

class CameraUndistorter:
    def __init__(self, calib_file=None):
        if calib_file is None:
            calib_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.json")
            
        self.calib_file = calib_file
        self.camera_matrix = np.array([[228.25, 0.0, 320.0], [0.0, 228.25, 180.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.12, 0.03, 0.0, 0.0, 0.0], dtype=np.float32)
        self.enabled = True
        self.load_calibration()

    def load_calibration(self):
        if os.path.exists(self.calib_file):
            try:
                with open(self.calib_file, 'r') as f:
                    cfg = json.load(f)
                    self.camera_matrix = np.array(cfg.get("camera_matrix", self.camera_matrix), dtype=np.float32)
                    self.dist_coeffs = np.array(cfg.get("dist_coeffs", self.dist_coeffs), dtype=np.float32)
                    self.enabled = cfg.get("undistort_enabled", True)
                print(f"[UNDISTORT] Loaded calibration matrix from {self.calib_file}")
            except Exception as e:
                print(f"[UNDISTORT ERROR] Failed to load calibration file: {e}")

    def undistort(self, frame):
        if not self.enabled or frame is None:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

class YoloObjectDetector:
    def __init__(self, min_area=400):
        self.min_area = min_area

    def detect_bounding_boxes(self, frame, hsv_cfg):
        """
        Extracts Bounding Boxes (x, y, w, h) for Green and Red WRO obstacle blocks
        """
        if frame is None:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        # Green Block Mask
        cg = hsv_cfg.get('green', {})
        lower_g = np.array([cg.get('h_min', 35), cg.get('s_min', 80), cg.get('v_min', 50)])
        upper_g = np.array([cg.get('h_max', 85), cg.get('s_max', 255), cg.get('v_max', 255)])
        mask_g = cv2.inRange(hsv, lower_g, upper_g)
        mask_g = cv2.erode(mask_g, None, iterations=2)
        mask_g = cv2.dilate(mask_g, None, iterations=2)

        # Pink Parking Wall Mask
        cp = hsv_cfg.get('pink', {})
        lower_p = np.array([cp.get('h_min', 140), cp.get('s_min', 35), cp.get('v_min', 70)])
        upper_p = np.array([cp.get('h_max', 170), cp.get('s_max', 150), cp.get('v_max', 255)])
        mask_p = cv2.inRange(hsv, lower_p, upper_p)
        mask_p = cv2.erode(mask_p, None, iterations=2)
        mask_p = cv2.dilate(mask_p, None, iterations=2)

        # Red Block Mask (with strict exclusion of pink pixels)
        cr1 = hsv_cfg.get('red1', {})
        cr2 = hsv_cfg.get('red2', {})
        l_r1 = np.array([cr1.get('h_min', 0), cr1.get('s_min', 155), cr1.get('v_min', 80)])
        u_r1 = np.array([cr1.get('h_max', 10), cr1.get('s_max', 255), cr1.get('v_max', 255)])
        l_r2 = np.array([cr2.get('h_min', 172), cr2.get('s_min', 155), cr2.get('v_min', 80)])
        u_r2 = np.array([cr2.get('h_max', 180), cr2.get('s_max', 255), cr2.get('v_max', 255)])
        m_r1 = cv2.inRange(hsv, l_r1, u_r1)
        m_r2 = cv2.inRange(hsv, l_r2, u_r2)
        mask_r = cv2.bitwise_or(m_r1, m_r2)
        # Strictly subtract pink from red to ensure 100% clean isolation
        mask_r = cv2.bitwise_and(mask_r, cv2.bitwise_not(mask_p))
        mask_r = cv2.erode(mask_r, None, iterations=2)
        mask_r = cv2.dilate(mask_r, None, iterations=2)

        for color_name, mask in [('green', mask_g), ('pink', mask_p), ('red', mask_r)]:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                cx = x + (w // 2)
                cy = y + (h // 2)

                aspect_ratio = float(w) / float(max(1, h))
                extent = float(area) / float(w * h + 1e-5)

                # ── GEOMETRIC SHAPE FILTERING FOR WRO STANDING PILLAR BLOCKS (RED & GREEN) ──
                if color_name in ['green', 'red']:
                    # 1. Aspect Ratio Filter: Standing blocks MUST be vertical (h >= w, aspect_ratio <= 1.25).
                    # Ground lines are wide horizontal rectangles (aspect_ratio > 1.25)
                    if aspect_ratio > 1.25:
                        continue

                    # 2. Bounding Box Max Width Filter: Blocks at 30cm-2m distance are <= 250px wide.
                    # Ground lines span 300px to 600px across the track width
                    if w > 250:
                        continue

                    # 3. Solidity / Extent Filter: Standing blocks fill >= 40% of their bounding box.
                    # Thin diagonal ground lines form low extent (< 0.40) inside bounding box
                    if extent < 0.40:
                        continue

                # Quality / confidence score
                confidence = min(1.0, float(area) / (w * h + 1e-5))

                # Double check color classification on ROI patch
                roi_hsv = hsv[y:y+h, x:x+w]
                if roi_hsv.size > 0:
                    med_h = np.median(roi_hsv[:, :, 0])
                    med_s = np.median(roi_hsv[:, :, 1])
                    # If ROI Hue is in pink range (135..171) and saturation is not ultra-deep red (< 155) -> force pink
                    if 135 <= med_h <= 171 and med_s < 155:
                        actual_color = 'pink'
                    else:
                        actual_color = color_name
                else:
                    actual_color = color_name

                detections.append({
                    "class": actual_color,
                    "bbox": (int(x), int(y), int(w), int(h)),
                    "cx": int(cx),
                    "cy": int(cy),
                    "area": float(area),
                    "confidence": round(confidence, 2),
                    "contour": c
                })

        return detections
