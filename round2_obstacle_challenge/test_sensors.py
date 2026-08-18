#!/usr/bin/env python3
"""
test_sensors.py – Test RPLiDAR C1 and Fifine K420 Camera on Raspberry Pi 5
"""

import os
import sys
import time
import serial

def test_camera():
    print("=" * 50)
    print("[1/2] Testing Fifine K420 Webcam (/dev/video0)...")
    try:
        import cv2
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("  [FAIL] Unable to open /dev/video0")
            return False
        
        # Set YUYV codec matching production wro_pi_system.py
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 368)
        
        # Warmup frames
        for _ in range(5):
            cap.read()
            time.sleep(0.05)
            
        ret, frame = cap.read()
        cap.release()
        
        if ret and frame is not None and frame.size > 0:
            h, w, c = frame.shape
            output_jpg = os.path.expanduser("~/pi_code/camera_test.jpg")
            cv2.imwrite(output_jpg, frame)
            print(f"  [SUCCESS] Camera captured live frame: {w}x{h} (channels: {c})")
            print(f"  [+] Saved test snapshot to: {output_jpg}")
            return True
        else:
            print("  [FAIL] Failed to read frame from camera")
            return False
    except Exception as e:
        print(f"  [ERROR] Camera test error: {e}")
        return False

def test_rplidar():
    print("=" * 50)
    print("[2/2] Testing RPLiDAR C1 (/dev/ttyUSB0 @ 460800 baud)...")
    port = "/dev/ttyUSB0"
    baud = 460800
    
    if not os.path.exists(port):
        print(f"  [FAIL] Device port {port} not found.")
        return False
        
    try:
        ser = serial.Serial(port, baud, timeout=1)
        ser.reset_input_buffer()
        
        # Send GET_INFO command (0xA5 0x50)
        ser.write(b'\xa5\x50')
        time.sleep(0.1)
        resp_info = ser.read(27)
        
        # Send GET_HEALTH command (0xA5 0x52)
        ser.write(b'\xa5\x52')
        time.sleep(0.1)
        resp_health = ser.read(10)
        
        ser.close()
        
        if len(resp_info) >= 27 and resp_info.startswith(b'\xa5\x5a'):
            model_id = resp_info[7]
            firmware_ver = f"{resp_info[9]}.{resp_info[8]}"
            serial_num = resp_info[11:27].hex()
            print(f"  [SUCCESS] RPLiDAR C1 Connected!")
            print(f"  [+] Model ID    : 0x{model_id:02X} (RPLiDAR C1)")
            print(f"  [+] Firmware    : v{firmware_ver}")
            print(f"  [+] Serial No.  : {serial_num}")
            
            if len(resp_health) >= 10 and resp_health.startswith(b'\xa5\x5a'):
                status_code = resp_health[7]
                status_str = "OK (0)" if status_code == 0 else f"Warning/Error ({status_code})"
                print(f"  [+] Health State: {status_str}")
            return True
        else:
            print(f"  [FAIL] Unexpected response from {port}: {resp_info.hex()}")
            return False
            
    except Exception as e:
        print(f"  [ERROR] RPLiDAR C1 test error: {e}")
        return False

def main():
    print("Starting Hardware Diagnostic Test on Raspberry Pi 5...\n")
    cam_ok = test_camera()
    lidar_ok = test_rplidar()
    
    print("\n" + "=" * 50)
    print("DIAGNOSTIC SUMMARY:")
    print(f"  - Camera (Fifine K420) : {'PASS [OK]' if cam_ok else 'FAIL'}")
    print(f"  - RPLiDAR C1 Scanner   : {'PASS [OK]' if lidar_ok else 'FAIL'}")
    print("=" * 50)

if __name__ == '__main__':
    main()
