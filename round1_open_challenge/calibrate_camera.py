#!/usr/bin/env python3
"""
calibrate_camera.py – Interactive Checkerboard Camera Calibration for 16:9 Wide-Angle Lens
Computes Camera Matrix K and Distortion Coefficients D to correct Barrel/Fisheye Distortion.
"""

import os
import sys
import time
import json
import numpy as np
import cv2

CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.json")

def calibrate_from_live_camera(pattern_size=(9, 6), square_size_mm=25.0, num_samples=15):
    print("=" * 60)
    print("  Checkerboard Camera Calibration (16:9 Widescreen Mode)")
    print(f"  Target Grid Inner Corners: {pattern_size[0]} x {pattern_size[1]}")
    print(f"  Target Samples Needed    : {num_samples}")
    print("=" * 60)

    # 3D points of real world checkerboard corners
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size_mm

    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360) # 16:9 aspect ratio

    if not cap.isOpened():
        print("[ERROR] Unable to open /dev/video0 for calibration.")
        return False

    collected = 0
    last_sample_time = time.time()

    try:
        while collected < num_samples:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            frame = cv2.resize(frame, (640, 360))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

            if found and (time.time() - last_sample_time > 1.2):
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                objpoints.append(objp)
                imgpoints.append(corners2)
                collected += 1
                last_sample_time = time.time()
                print(f"  [+] Captured sample {collected}/{num_samples}")

                cv2.drawChessboardCorners(frame, pattern_size, corners2, found)

            time.sleep(0.03)

        cap.release()

        print("\n[*] Calculating Camera Matrix K and Distortion Coefficients D...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (640, 360), None, None)

        if ret:
            print("[SUCCESS] Calibration Complete!")
            calib_data = {
                "aspect_ratio": "16:9",
                "resolution": [640, 360],
                "camera_matrix": mtx.tolist(),
                "dist_coeffs": dist.ravel().tolist(),
                "undistort_enabled": True
            }

            with open(CALIB_PATH, 'w') as f:
                json.dump(calib_data, f, indent=2)

            print(f"[+] Saved updated calibration parameters to: {CALIB_PATH}")
            print(f"  - fx: {mtx[0,0]:.2f}, fy: {mtx[1,1]:.2f}")
            print(f"  - cx: {mtx[0,2]:.2f}, cy: {mtx[1,2]:.2f}")
            print(f"  - k1: {dist[0,0]:.4f}, k2: {dist[0,1]:.4f}")
            return True
        else:
            print("[FAIL] cv2.calibrateCamera failed.")
            return False

    except Exception as e:
        print(f"[ERROR] Calibration error: {e}")
        return False
    finally:
        if cap and cap.isOpened():
            cap.release()

if __name__ == '__main__':
    grid_w = int(sys.argv[1]) if len(sys.argv) > 1 else 9
    grid_h = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    calibrate_from_live_camera(pattern_size=(grid_w, grid_h))
