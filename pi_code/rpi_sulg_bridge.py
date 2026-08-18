#!/usr/bin/env python3
"""
rpi_sulg_bridge.py  –  Raspberry Pi 5 Python Bridge for ESP32 Sulg Controller

Interfacing Raspberry Pi 5 (LiDAR + Camera) with ESP32 over USB Serial.

Usage:
  python3 rpi_sulg_bridge.py
"""

import serial
import time
import threading

class SulgBridge:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False

        # Live Telemetry State (Updated at 20 Hz from ESP32)
        self.x = 0            # X coordinate in mm
        self.y = 0            # Y coordinate in mm
        self.yaw = 0.0        # Fused heading in degrees (0-360)
        self.dist = 0         # Cumulative distance in mm
        self.speed = 0        # Speed in mm/s
        self.sensors_ok = True

    def connect(self):
        """Connect to ESP32 Serial Port."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2.0)  # Wait for ESP32 serial reset
            self.running = True
            self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
            self.read_thread.start()
            print(f"[OK] Connected to ESP32 on {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            print(f"[ERROR] Could not connect to {self.port}: {e}")
            return False

    def _read_serial_loop(self):
        """Background thread reading telemetry packets from ESP32."""
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("ODOM,"):
                    parts = line.split(',')
                    if len(parts) >= 7:
                        self.x = int(parts[1])
                        self.y = int(parts[2])
                        self.yaw = float(parts[3])
                        self.dist = int(parts[4])
                        self.speed = int(parts[5])
                        self.sensors_ok = bool(int(parts[6]))
            except Exception:
                pass

    def drive(self, speed, steer_angle=110):
        """
        Send drive command to ESP32.
        :param speed: -255 to 255
        :param steer_angle: 50 (Full Right) to 170 (Full Left), 110 (Center)
        """
        if self.ser and self.ser.is_open:
            cmd = f"D {speed} {steer_angle}\n"
            self.ser.write(cmd.encode('utf-8'))

    def turn_closed_loop(self, turn_angle_deg, speed=55):
        """
        Execute precision closed-loop turn on ESP32.
        :param turn_angle_deg: Positive for Left (+90), Negative for Right (-90)
        :param speed: Turn motor speed (default 55)
        """
        if self.ser and self.ser.is_open:
            cmd = f"T {turn_angle_deg} {speed}\n"
            self.ser.write(cmd.encode('utf-8'))

    def stop(self):
        """Emergency stop motor and center steering."""
        if self.ser and self.ser.is_open:
            self.ser.write(b"S\n")

    def reset_odometry(self):
        """Reset (X, Y, Yaw, Dist) origin on ESP32."""
        if self.ser and self.ser.is_open:
            self.ser.write(b"R\n")

    def disconnect(self):
        """Close serial connection."""
        self.stop()
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[INFO] ESP32 Bridge Disconnected.")

# ── Demo / Standalone Test Program ─────────────────────────
if __name__ == '__main__':
    bot = SulgBridge(port='/dev/ttyUSB0', baudrate=115200)

    if bot.connect():
        try:
            print("\n--- Telemetry Live Stream Test ---")
            for _ in range(20):
                print(f"Pos: ({bot.x} mm, {bot.y} mm) | Yaw: {bot.yaw}° | Dist: {bot.dist} mm | Speed: {bot.speed} mm/s")
                time.sleep(0.2)

            print("\n--- Testing Drive Command ---")
            bot.drive(speed=55, steer_angle=110) # Drive straight
            time.sleep(2.0)
            bot.stop()

            print("\n--- Testing 90° Turn Command ---")
            bot.turn_closed_loop(turn_angle_deg=90.0, speed=55) # 90 deg Left turn
            time.sleep(4.0)

            print("\n--- Final Location ---")
            print(f"Final Pos: ({bot.x} mm, {bot.y} mm) | Final Yaw: {bot.yaw}°")

        except KeyboardInterrupt:
            print("\nStopping bot...")
        finally:
            bot.disconnect()
