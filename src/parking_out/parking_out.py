#!/usr/bin/env python3
"""
parking_out.py – WRO Autonomous Exit Parking Maneuver
Step 1: Move backward 3 cm (30 mm)
Step 2: Turn steering to leftmost (60 PWM)
Step 3: Move forward 10 cm (100 mm)
"""

import os
import sys
import time
import json
import serial

# Detect active ESP32 Port
ESP32_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_56BA018173-if00"
BAUD_RATE = 115200

# Try loading from configuration if available
CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "round1_open_challenge_odometry", "hsv_config.json")
if os.path.exists(CONFIG_PATH):
    try:
        with open(CONFIG_PATH, 'r') as f:
            cfg = json.load(f)
            ESP32_PORT = cfg.get("hardware", {}).get("arduino_port", ESP32_PORT)
            BAUD_RATE = cfg.get("hardware", {}).get("arduino_baud", BAUD_RATE)
    except Exception as e:
        print(f"[WARN] Failed to parse config file: {e}")

def run_parking_out():
    print(f"[*] Connecting to ESP32 on {ESP32_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Could not open port {ESP32_PORT}: {e}")
        sys.exit(1)

    time.sleep(1.0) # Wait for serial bridge stabilization
    ser.reset_input_buffer()

    # Reset ESP32 odometry to zero
    print("[*] Resetting ESP32 odometry...")
    ser.write(b"R\n")
    time.sleep(0.1)
    ser.reset_input_buffer()

    state = "BACK_3CM"
    start_dist = None
    exit_done = False

    print("[START] Executing Autonomous Parking Out Sequence...")

    while not exit_done:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith("ODOM,"):
                continue

            parts = line.split(',')
            if len(parts) < 6:
                continue

            esp_dist = int(parts[4]) # Cumulative distance in mm

            # State 1: Move backward 3cm (30 mm)
            if state == "BACK_3CM":
                if start_dist is None:
                    start_dist = esp_dist
                    print(f"  [+] State: Backing up 3cm. Start distance: {start_dist}mm")

                dist_diff = abs(esp_dist - start_dist)
                if dist_diff < 30:
                    # Drive backward (speed = -60, steer = center 110)
                    ser.write(b"D -60 110\n")
                else:
                    print(f"  [+] Completed backing up 3cm (Traveled: {dist_diff}mm)")
                    ser.write(b"S\n") # Stop
                    time.sleep(0.3)
                    start_dist = None
                    state = "STEER_LEFT"

            # State 2: Set steering to leftmost
            elif state == "STEER_LEFT":
                print("  [+] State: Setting steering leftmost (60)")
                # Steer Leftmost (60), speed = 0
                ser.write(b"D 0 60\n")
                time.sleep(0.5) # Wait for servo to physically steer
                state = "FORWARD_10CM"

            # State 3: Move forward 10cm (100 mm)
            elif state == "FORWARD_10CM":
                if start_dist is None:
                    start_dist = esp_dist
                    print(f"  [+] State: Driving forward 10cm. Start distance: {start_dist}mm")

                dist_diff = abs(esp_dist - start_dist)
                if dist_diff < 100:
                    # Drive forward (speed = 65, steer = leftmost 60)
                    ser.write(b"D 65 60\n")
                else:
                    print(f"  [+] Completed driving forward 10cm (Traveled: {dist_diff}mm)")
                    ser.write(b"S\n") # Stop
                    time.sleep(0.3)
                    state = "DONE"

            # State 4: Finished sequence
            elif state == "DONE":
                print("[SUCCESS] Parking Out Sequence Complete!")
                ser.write(b"S\n") # Stop and center
                exit_done = True

        except KeyboardInterrupt:
            print("\n[ABORT] User cancelled sequence.")
            ser.write(b"S\n")
            break
        except Exception as e:
            print(f"[ERROR] Loop error: {e}")
            ser.write(b"S\n")
            break

    ser.close()

if __name__ == '__main__':
    run_parking_out()
