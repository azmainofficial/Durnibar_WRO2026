#!/usr/bin/env python3
"""
test_planner.py – Multi-Scenario Verification & Benchmark for FastAckermannTrajectoryOptimizer
"""

import math
import time
import numpy as np
from optimal_planner import FastAckermannTrajectoryOptimizer

def test_straight_corridor():
    print("\n--- Test 1: Straight Corridor ---")
    planner = FastAckermannTrajectoryOptimizer()
    
    # Left wall at y = +0.8m, Right wall at y = -0.8m
    lidar_pts = []
    for x in np.linspace(0.3, 3.0, 30):
        # Left
        lidar_pts.append((math.degrees(math.atan2(0.8, x)), math.hypot(x, 0.8) * 1000.0))
        # Right
        lidar_pts.append(((360.0 + math.degrees(math.atan2(-0.8, x))) % 360.0, math.hypot(x, -0.8) * 1000.0))
        
    res = planner.plan_optimal_trajectory(
        lidar_points=lidar_pts,
        towers=[],
        base_speed_pwm=120,
        left_wall_m=0.8,
        right_wall_m=0.8,
        preferred_angle_deg=0.0,
        challenge_mode="OPEN_CHALLENGE"
    )
    
    print(f"Steer Angle: {res['steer_deg']}° (Expected ~0°)")
    print(f"Steer PWM: {res['steer_pwm']} (Expected ~110)")
    print(f"Speed: {res['target_speed_pwm']} PWM (Expected ~120)")
    print(f"Latency: {res['latency_ms']} ms")
    assert abs(res['steer_deg']) <= 3.0, f"Straight steer deviation too large: {res['steer_deg']}"
    print("[PASS] Straight Corridor PASSED")

def test_green_pillar_evasion():
    print("\n--- Test 2: Green Pillar Evasion (Must Pass on the LEFT) ---")
    planner = FastAckermannTrajectoryOptimizer()
    
    # Corridor with a Green Pillar right in the middle at x=1.0m, y=0.0m
    lidar_pts = []
    for x in np.linspace(0.3, 3.0, 20):
        lidar_pts.append((math.degrees(math.atan2(0.8, x)), math.hypot(x, 0.8) * 1000.0))
        lidar_pts.append(((360.0 + math.degrees(math.atan2(-0.8, x))) % 360.0, math.hypot(x, -0.8) * 1000.0))
    
    # Add Green Tower cluster
    towers = [
        {"x_m": 1.0, "y_m": 0.0, "dist_m": 1.0, "angle_deg": 0.0, "color": "green"}
    ]
    
    res = planner.plan_optimal_trajectory(
        lidar_points=lidar_pts,
        towers=towers,
        base_speed_pwm=100,
        left_wall_m=0.8,
        right_wall_m=0.8,
        preferred_angle_deg=0.0,
        challenge_mode="OBSTACLE_CHALLENGE"
    )
    
    print(f"Steer Angle: {res['steer_deg']}° (Expected Positive / Left turn > 0)")
    print(f"Steer PWM: {res['steer_pwm']} (Expected < 110)")
    print(f"Speed: {res['target_speed_pwm']} PWM")
    print(f"Latency: {res['latency_ms']} ms")
    assert res['steer_deg'] > 2.0, f"Green pillar should force left steer, got: {res['steer_deg']}"
    assert res['steer_pwm'] < 110, f"Servo PWM for Left turn should be < 110, got: {res['steer_pwm']}"
    print("[PASS] Green Pillar Evasion PASSED")

def test_red_pillar_evasion():
    print("\n--- Test 3: Red Pillar Evasion (Must Pass on the RIGHT) ---")
    planner = FastAckermannTrajectoryOptimizer()
    
    # Corridor with a Red Pillar right in the middle at x=1.0m, y=0.0m
    lidar_pts = []
    for x in [0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0]:
        lidar_pts.append((math.degrees(math.atan2(0.9, x)), math.hypot(x, 0.9) * 1000.0))
        lidar_pts.append(((360.0 + math.degrees(math.atan2(-0.9, x))) % 360.0, math.hypot(x, -0.9) * 1000.0))
    
    # Add Red Tower cluster at x=1.0m, y=0.0m
    towers = [
        {"x_m": 1.0, "y_m": 0.0, "dist_m": 1.0, "angle_deg": 0.0, "color": "red"}
    ]
    
    res = planner.plan_optimal_trajectory(
        lidar_points=lidar_pts,
        towers=towers,
        base_speed_pwm=100,
        left_wall_m=0.8,
        right_wall_m=0.8,
        preferred_angle_deg=0.0,
        challenge_mode="OBSTACLE_CHALLENGE"
    )
    
    print(f"Steer Angle: {res['steer_deg']}° (Expected Negative / Right turn < 0)")
    print(f"Steer PWM: {res['steer_pwm']} (Expected > 110)")
    print(f"Speed: {res['target_speed_pwm']} PWM")
    print(f"Latency: {res['latency_ms']} ms")
    assert res['steer_deg'] < -2.0, f"Red pillar should force right steer, got: {res['steer_deg']}"
    assert res['steer_pwm'] > 110, f"Servo PWM for Right turn should be > 110, got: {res['steer_pwm']}"
    print("[PASS] Red Pillar Evasion PASSED")

def test_latency_benchmark():
    print("\n--- Test 4: 500-Cycle High-Frequency Latency Benchmark ---")
    planner = FastAckermannTrajectoryOptimizer()
    
    # Realistic 72-point scan batch (5 deg resolution)
    lidar_pts = []
    for a in range(0, 360, 5):
        lidar_pts.append((float(a), 1200.0))
    towers = [
        {"x_m": 0.8, "y_m": -0.2, "dist_m": 0.82, "angle_deg": 346.0, "color": "red"}
    ]
    
    times = []
    for _ in range(500):
        t0 = time.perf_counter()
        res = planner.plan_optimal_trajectory(
            lidar_points=lidar_pts,
            towers=towers,
            base_speed_pwm=110,
            challenge_mode="OBSTACLE_CHALLENGE"
        )
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
        
    avg_ms = np.mean(times)
    p95_ms = np.percentile(times, 95)
    max_ms = np.max(times)
    print(f"500 Cycles Benchmark: Avg = {avg_ms:.2f} ms | 95th Percentile = {p95_ms:.2f} ms | Max = {max_ms:.2f} ms")
    assert avg_ms < 6.0, f"Average latency exceeds 6ms target: {avg_ms:.2f} ms"
    print("[PASS] Latency Benchmark PASSED")

if __name__ == '__main__':
    print("==================================================")
    print("  RUNNING OPTIMAL PATH PLANNER COMPREHENSIVE TESTS")
    print("==================================================")
    test_straight_corridor()
    test_green_pillar_evasion()
    test_red_pillar_evasion()
    test_latency_benchmark()
    print("\nALL VERIFICATION TESTS PASSED SUCCESSFULLY!")
