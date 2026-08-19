#!/usr/bin/env python3
"""
optimal_planner.py – Ultra-Fast Vectorized Ackermann Optimal Path Planner & Trajectory Optimizer
Designed for high-speed, sub-3ms autonomous navigation on Raspberry Pi 5 without ROS2.
"""

import time
import math
import numpy as np

class FastAckermannTrajectoryOptimizer:
    def __init__(self,
                 wheelbase_m=0.20,
                 robot_width_m=0.22,
                 robot_length_m=0.28,
                 max_steer_deg=30.0,
                 center_steer_pwm=110,
                 num_candidates=21,
                 horizon_m=1.8,
                 num_steps=25):
        """
        :param wheelbase_m: Distance between front and rear axles (m)
        :param robot_width_m: Total vehicle width including wheels (m)
        :param robot_length_m: Total vehicle length (m)
        :param max_steer_deg: Maximum wheel steering deflection (+/- deg)
        :param center_steer_pwm: Servo PWM angle for driving straight
        :param num_candidates: Number of steering trajectory rollouts to evaluate
        :param horizon_m: Forward prediction horizon (m)
        :param num_steps: Number of discrete integration steps along each rollout
        """
        self.wheelbase = wheelbase_m
        self.robot_width = robot_width_m
        self.robot_half_width = robot_width_m / 2.0
        self.robot_length = robot_length_m
        self.max_steer_deg = max_steer_deg
        self.center_steer = center_steer_pwm
        self.num_candidates = num_candidates
        self.horizon_m = horizon_m
        self.num_steps = num_steps

        # Tunable cost weights
        self.weights = {
            "w_clearance": 7.0,       # Strict repulsion from walls and obstacles
            "w_progress": 2.0,        # Heading towards the furthest open corridor
            "w_wro_rules": 12.0,      # Strict rule enforcement (Green=Left, Red=Right)
            "w_smooth": 0.8,          # Steering rate penalty to prevent wobble
            "w_centering": 2.5,       # Track centerline bias
            "safety_margin_m": 0.30,  # Desired clearance buffer around robot
            "max_lat_accel": 2.2      # Maximum lateral cornering acceleration (m/s^2)
        }

        # Candidate steering angles (-max_steer_deg to +max_steer_deg)
        self.steer_angles_deg = np.linspace(-self.max_steer_deg, self.max_steer_deg, self.num_candidates)
        self.steer_angles_rad = np.radians(self.steer_angles_deg)

        # Precompute normalized kinematic rollouts for unit speed
        self._precompute_trajectory_templates()

        # State tracking
        self.last_selected_steer_deg = 0.0
        self.last_plan_time = time.perf_counter()
        self.last_latency_ms = 0.0
        self.last_error = 0.0

    def _precompute_trajectory_templates(self):
        """
        Precompute normalized 2D trajectory coordinate rollouts for all candidate steering angles.
        Shape: (num_candidates, num_steps, 2) where [:, :, 0] is X (forward), [:, :, 1] is Y (left)
        """
        dt = 0.05
        v = 1.0  # Unit speed template
        step_dist = self.horizon_m / float(self.num_steps)
        dt_eff = step_dist / v

        self.template_trajectories = np.zeros((self.num_candidates, self.num_steps, 2), dtype=np.float32)
        self.template_headings = np.zeros((self.num_candidates, self.num_steps), dtype=np.float32)

        for i, delta in enumerate(self.steer_angles_rad):
            x, y, theta = 0.0, 0.0, 0.0
            for s in range(self.num_steps):
                x += v * math.cos(theta) * dt_eff
                y += v * math.sin(theta) * dt_eff
                theta += (v / self.wheelbase) * math.tan(delta) * dt_eff
                self.template_trajectories[i, s, 0] = x
                self.template_trajectories[i, s, 1] = y
                self.template_headings[i, s] = theta

        # Precompute pre-rounded Python lists for instant telemetry serialization (<0.05ms)
        self.template_trajectories_list = [
            [[round(float(p[0]), 3), round(float(p[1]), 3)] for p in self.template_trajectories[i]]
            for i in range(self.num_candidates)
        ]

    def update_config(self, cfg_dict):
        """Live update tunable planner weights from web dashboard or config file."""
        for k in self.weights:
            if k in cfg_dict:
                try:
                    self.weights[k] = float(cfg_dict[k])
                except (ValueError, TypeError):
                    pass

    def plan_optimal_trajectory(self,
                                lidar_points,
                                towers=None,
                                current_speed_pwm=110,
                                base_speed_pwm=110,
                                left_wall_m=0.8,
                                right_wall_m=0.8,
                                preferred_angle_deg=0.0,
                                challenge_mode="OPEN_CHALLENGE"):
        """
        Computes the collision-free, curvature-optimal trajectory in < 3 ms.

        :param lidar_points: List of (angle_deg, dist_mm) from 360-degree LiDAR
        :param towers: List of detected fused towers [{'x_m', 'y_m', 'dist_m', 'angle_deg', 'color'}, ...]
        :param current_speed_pwm: Current motor speed PWM
        :param base_speed_pwm: Target nominal speed PWM
        :param left_wall_m: Smoothed left wall distance (m)
        :param right_wall_m: Smoothed right wall distance (m)
        :param preferred_angle_deg: Fallback guidance angle from gap finder
        :param challenge_mode: "OPEN_CHALLENGE", "OBSTACLE_CHALLENGE", "PARKING", "IDLE"
        :return: dict with {
            'steer_pwm': int (50..170),
            'steer_deg': float,
            'target_speed_pwm': int,
            'optimal_path': list of [x, y],
            'candidate_paths': list of dict,
            'latency_ms': float,
            'cost': float,
            'safe': bool
        }
        """
        t_start = time.perf_counter()
        if towers is None:
            towers = []

        # 1. Convert LiDAR front sector (-100 deg to +100 deg) to 2D Cartesian obstacle points (x_m, y_m)
        if lidar_points:
            raw_arr = np.asarray(lidar_points, dtype=np.float32)
            angles = raw_arr[:, 0]
            dists = raw_arr[:, 1]
            angles_rel = np.where(angles <= 180.0, angles, angles - 360.0)
            # Ignore points closer than 160mm (vehicle's own chassis / wires / bumper)
            mask = (dists >= 160.0) & (dists <= 3500.0) & (angles_rel >= -100.0) & (angles_rel <= 100.0)
            
            if np.any(mask):
                valid_rad = np.radians(angles_rel[mask])
                valid_d_m = dists[mask] * 0.001
                ox = valid_d_m * np.cos(valid_rad)
                oy = valid_d_m * np.sin(valid_rad)
                obs_arr = np.column_stack((ox, oy))
            else:
                obs_arr = np.empty((0, 2), dtype=np.float32)
        else:
            obs_arr = np.empty((0, 2), dtype=np.float32)

        # 2. Evaluate All Candidate Trajectories in Parallel (Fully Vectorized)
        num_cands = self.num_candidates
        costs = np.zeros(num_cands, dtype=np.float32)
        valid_mask = np.ones(num_cands, dtype=bool)

        w_clearance = self.weights["w_clearance"]
        w_progress  = self.weights["w_progress"]
        w_wro       = self.weights["w_wro_rules"]
        w_smooth    = self.weights["w_smooth"]
        w_center    = self.weights["w_centering"]
        safety_buf  = self.weights["safety_margin_m"]
        crit_radius = self.robot_half_width + 0.03  # Absolute collision limit (m)

        # Target progress angle in radians
        pref_rad = math.radians(preferred_angle_deg)
        pref_dir = np.array([math.cos(pref_rad), math.sin(pref_rad)], dtype=np.float32)

        # Corridor centering bias
        track_center_offset = 0.0
        if 0.1 < left_wall_m < 2.5 and 0.1 < right_wall_m < 2.5:
            track_center_offset = (left_wall_m - right_wall_m) * 0.5

        # ── A. Collision & Clearance Cost with LiDAR Obstacles (Vectorized across all candidates) ──
        if len(obs_arr) > 0:
            # templates: (C, N, 2), obs_arr: (M, 2)
            # diff: (C, N, M, 2)
            diff = self.template_trajectories[:, :, np.newaxis, :] - obs_arr[np.newaxis, np.newaxis, :, :]
            dists_sq = np.sum(diff ** 2, axis=3)  # (C, N, M)
            min_dists_per_pt = np.sqrt(np.min(dists_sq, axis=2))  # (C, N)
            min_clearance = np.min(min_dists_per_pt, axis=1)  # (C,)

            # Mark collisions
            collision_mask = min_clearance < crit_radius
            valid_mask[collision_mask] = False
            costs[collision_mask] = 1e6

            # Compute clearance cost for non-collision paths
            clearance_costs = np.sum(np.exp(-min_dists_per_pt / (crit_radius + safety_buf)), axis=1)
            costs[~collision_mask] += w_clearance * clearance_costs[~collision_mask]

        # ── B. Progress & Gap Alignment Cost (Vectorized) ──
        end_pts = self.template_trajectories[:, -1, :]  # (C, 2)
        end_norms = np.linalg.norm(end_pts, axis=1, keepdims=True) + 1e-6
        norm_end_pts = end_pts / end_norms
        cos_sims = np.dot(norm_end_pts, pref_dir)  # (C,)
        costs[valid_mask] += w_progress * (1.0 - cos_sims[valid_mask]) * 10.0

        # ── C. Smoothness / Steering Rate Penalty (Vectorized) ──
        steer_changes = np.abs(self.steer_angles_deg - self.last_selected_steer_deg)
        costs[valid_mask] += w_smooth * (steer_changes[valid_mask] / self.max_steer_deg) ** 1.5

        # ── D. Lane / Corridor Centering & Emergency Wall Repulsion (Vectorized) ──
        avg_traj_y = np.mean(self.template_trajectories[:, :, 1], axis=1)  # (C,)
        costs[valid_mask] += w_center * np.abs(avg_traj_y[valid_mask] - track_center_offset) * 3.0

        # Hard proximity repulsion if getting too close to either side wall or inner box corner
        if 0.05 < left_wall_m < 0.40:
            # Dangerously close to LEFT wall / inner corner -> penalize left steer (>0) and strongly favor right steer (<0)
            left_wall_danger = (0.40 - left_wall_m) / 0.40
            steer_bias = np.where(self.steer_angles_deg > 0, self.steer_angles_deg * 3.0, -self.steer_angles_deg * 2.0)
            costs[valid_mask] += 160.0 * left_wall_danger * (steer_bias[valid_mask] / self.max_steer_deg)

        if 0.05 < right_wall_m < 0.40:
            # Dangerously close to RIGHT wall / inner corner -> penalize right steer (<0) and strongly favor left steer (>0)
            right_wall_danger = (0.40 - right_wall_m) / 0.40
            steer_bias = np.where(self.steer_angles_deg < 0, -self.steer_angles_deg * 3.0, self.steer_angles_deg * 2.0)
            costs[valid_mask] += 160.0 * right_wall_danger * (steer_bias[valid_mask] / self.max_steer_deg)

        # ── E. WRO Obstacle Pillar Rules (Color-Specific Evasion) ──
        if challenge_mode == "OBSTACLE_CHALLENGE" and towers:
            for tower in towers:
                t_color = tower.get('color', 'unknown')
                t_dist = tower.get('dist_m', 99.0)
                t_x = tower.get('x_m', 0.0)
                t_y = tower.get('y_m', 0.0)

                # Only consider towers within 1.6m in front of the vehicle
                if 0.1 < t_dist < 1.6 and t_x > 0.05:
                    t_pos = np.array([t_x, t_y], dtype=np.float32)
                    t_diff = self.template_trajectories - t_pos[np.newaxis, np.newaxis, :]  # (C, N, 2)
                    t_dists = np.sqrt(np.sum(t_diff ** 2, axis=2))  # (C, N)
                    min_idx = np.argmin(t_dists, axis=1)  # (C,)
                    min_t_dists = np.min(t_dists, axis=1)  # (C,)

                    # Y position of trajectory at closest point to tower (+ is LEFT, - is RIGHT)
                    traj_y_at_tower = self.template_trajectories[np.arange(num_cands), min_idx, 1]  # (C,)

                    if t_color == 'green':
                        # GREEN: MUST pass to robot's LEFT (positive Y axis)
                        # Correct: traj_y_at_tower >= t_y + LATERAL_BUF (robot is left of pillar)
                        # Wrong:   traj_y_at_tower <  t_y + LATERAL_BUF (robot center-right of pillar)
                        LATERAL_BUF = 0.25
                        wrong_side = (traj_y_at_tower < t_y + LATERAL_BUF) & valid_mask
                        costs[wrong_side] += w_wro * 150.0 * (1.0 / (min_t_dists[wrong_side] + 0.05))
                        correct_side = (~wrong_side) & valid_mask
                        costs[correct_side] -= w_wro * 20.0

                    elif t_color == 'red':
                        # RED: MUST pass to robot's RIGHT (negative Y axis)
                        # Correct: traj_y_at_tower <= t_y - LATERAL_BUF (robot is right of pillar)
                        # Wrong:   traj_y_at_tower >  t_y - LATERAL_BUF (robot center-left of pillar)
                        LATERAL_BUF = 0.25
                        wrong_side = (traj_y_at_tower > t_y - LATERAL_BUF) & valid_mask
                        costs[wrong_side] += w_wro * 150.0 * (1.0 / (min_t_dists[wrong_side] + 0.05))
                        correct_side = (~wrong_side) & valid_mask
                        costs[correct_side] -= w_wro * 20.0

                    elif t_color == 'pink':
                        # Absolute collision hazard: heavy penalty on any path approaching within 0.40m
                        too_close = (min_t_dists < 0.40) & valid_mask
                        costs[too_close] += w_wro * 180.0 * (1.0 / (min_t_dists[too_close] + 0.05))

                    # Hard proximity safety buffer: any trajectory passing within 0.28m of ANY pillar gets severe penalty
                    too_close_pillar = (min_t_dists < 0.28) & valid_mask
                    costs[too_close_pillar] += 250.0 * (0.28 - min_t_dists[too_close_pillar])

        # 3. Select the Optimal Candidate Trajectory
        if np.any(valid_mask):
            valid_indices = np.where(valid_mask)[0]
            best_idx = valid_indices[np.argmin(costs[valid_indices])]
            best_cost = float(costs[best_idx])
            safe = True
        else:
            # Fallback when all paths are close to obstacles: steer towards MAXIMUM clearance path
            if len(obs_arr) > 0 and 'min_clearance' in locals():
                best_idx = int(np.argmax(min_clearance))
            else:
                best_idx = int(np.argmax(cos_sims))
            best_cost = float(costs[best_idx])
            safe = False

        selected_steer_deg = float(self.steer_angles_deg[best_idx])
        self.last_selected_steer_deg = selected_steer_deg

        # Map steer deflection (e.g. -30..+30) to servo PWM (50..170, center 110)
        # In this chassis: Left turn (>0 deg) decreases angle towards 50, Right turn (<0 deg) increases towards 170
        steer_pwm = int(round(self.center_steer - selected_steer_deg * (60.0 / self.max_steer_deg)))
        steer_pwm = max(50, min(170, steer_pwm))

        # 4. Dynamic Cornering Speed Profiling
        # Curvature: kappa = tan(delta) / L
        delta_rad = math.radians(abs(selected_steer_deg))
        curvature = math.tan(delta_rad) / self.wheelbase
        speed_factor = 1.0 / (1.0 + 1.4 * curvature)

        # Scale speed based on road curvature (slow into turns ~40-48, straights ~55)
        # Strictly clamp maximum speed to base_speed_pwm (e.g. 55 max)
        target_speed_pwm = int(round(base_speed_pwm * max(0.65, speed_factor)))
        target_speed_pwm = max(35, min(int(base_speed_pwm), target_speed_pwm))
        if not safe:
            target_speed_pwm = max(32, int(target_speed_pwm * 0.7))

        # Instant zero-alloc trajectory formatting for web visualizer
        optimal_path = self.template_trajectories_list[best_idx]

        # Send subset of 7 representative candidate paths to keep telemetry lightweight (< 1KB)
        sample_indices = np.linspace(0, num_cands - 1, 7, dtype=int)
        candidate_paths = [
            {
                "steer_deg": round(float(self.steer_angles_deg[idx]), 1),
                "path": self.template_trajectories_list[idx],
                "cost": round(float(costs[idx]), 2),
                "valid": bool(valid_mask[idx])
            }
            for idx in sample_indices
        ]

        t_end = time.perf_counter()
        self.last_latency_ms = round((t_end - t_start) * 1000.0, 2)

        return {
            "steer_pwm": steer_pwm,
            "steer_deg": round(selected_steer_deg, 2),
            "target_speed_pwm": target_speed_pwm,
            "optimal_path": optimal_path,
            "candidate_paths": candidate_paths,
            "latency_ms": self.last_latency_ms,
            "cost": round(best_cost, 2),
            "safe": safe
        }

    def plan_virtual_corridor_pid(self,
                                  towers,
                                  left_wall_m,
                                  right_wall_m,
                                  base_speed_pwm=55,
                                  dt=0.05):
        """
        Virtual Corridor Target PID Controller:
        - GREEN block: Target corridor = halfway between Green block's left edge (t_y + 0.25m) and Left Wall (+left_wall_m).
        - RED block:   Target corridor = halfway between Red block's right edge (t_y - 0.25m) and Right Wall (-right_wall_m).
        - NO block:    Target corridor = halfway between Left Wall (+left_wall_m) and Right Wall (-right_wall_m).

        Computes PID lateral error e_y = y_target - 0, and calculates steering servo PWM (50..170).
        """
        t_start = time.perf_counter()

        # Find nearest active color block ahead (within 1.8m)
        target_block = None
        min_x = 99.0

        if towers:
            for t in towers:
                color = t.get('color', 'unknown')
                tx = t.get('x_m', 99.0)
                ty = t.get('y_m', 0.0)
                if color in ['green', 'red'] and 0.10 < tx < 1.8 and abs(ty) < 0.60:
                    if tx < min_x:
                        min_x = tx
                        target_block = t

        # Sanitize wall distances
        l_wall = left_wall_m if (0.05 < left_wall_m < 2.5) else 0.80
        r_wall = right_wall_m if (0.05 < right_wall_m < 2.5) else 0.80

        # Boundary positions in robot body frame (+y is left, -y is right)
        y_left_wall = +l_wall
        y_right_wall = -r_wall

        action_name = "CORRIDOR_PID_CENTER"

        if target_block:
            t_color = target_block.get('color')
            ty = target_block.get('y_m', 0.0)
            BLOCK_MARGIN = 0.25  # Clearance buffer from pillar center (m)

            if t_color == 'green':
                # GREEN BLOCK: Pass on LEFT.
                # Inner boundary: Green block left edge = ty + BLOCK_MARGIN
                # Outer boundary: Left wall = +l_wall
                y_inner = ty + BLOCK_MARGIN
                y_outer = y_left_wall
                # Target is halfway between Green block left edge and Left Wall
                y_target = (y_inner + y_outer) / 2.0
                action_name = f"VIRTUAL_PATH_GREEN_LEFT (target={y_target:.2f}m)"

            elif t_color == 'red':
                # RED BLOCK: Pass on RIGHT.
                # Inner boundary: Red block right edge = ty - BLOCK_MARGIN
                # Outer boundary: Right wall = -r_wall
                y_inner = ty - BLOCK_MARGIN
                y_outer = y_right_wall
                # Target is halfway between Red block right edge and Right Wall
                y_target = (y_inner + y_outer) / 2.0
                action_name = f"VIRTUAL_PATH_RED_RIGHT (target={y_target:.2f}m)"
            else:
                y_target = (y_left_wall + y_right_wall) / 2.0
        else:
            # NO BLOCK: Center between track walls
            y_target = (y_left_wall + y_right_wall) / 2.0

        # PID Lateral Error: error_y = y_target - robot_y (robot_y = 0 in body frame)
        error_y = y_target - 0.0

        # PID gains
        Kp = 45.0
        Kd = 8.0

        # Derivative calculation
        d_error = (error_y - self.last_error) / max(0.01, dt)
        self.last_error = error_y

        # Steering adjustment in PWM:
        # positive y_target (left) -> steer LEFT (PWM < 110)
        # negative y_target (right) -> steer RIGHT (PWM > 110)
        steer_adj = int(round(-Kp * error_y - Kd * d_error))
        steer_adj = max(-55, min(55, steer_adj))

        steer_pwm = self.center_steer + steer_adj
        steer_pwm = max(55, min(165, steer_pwm))

        # Speed scaling: full speed on straight targets, slow into tight turns
        steer_abs_deg = abs(steer_pwm - 110) * (30.0 / 60.0)
        speed_factor = max(0.65, 1.0 - (steer_abs_deg / 30.0) * 0.35)
        target_speed_pwm = int(round(base_speed_pwm * speed_factor))

        t_end = time.perf_counter()
        latency_ms = round((t_end - t_start) * 1000.0, 2)

        return {
            "steer_pwm": steer_pwm,
            "steer_deg": round((steer_pwm - 110) * (30.0 / 60.0), 1),
            "target_speed_pwm": target_speed_pwm,
            "y_target": round(y_target, 3),
            "action": action_name,
            "latency_ms": latency_ms
        }

if __name__ == '__main__':
    print("=== Testing FastAckermannTrajectoryOptimizer ===")
    planner = FastAckermannTrajectoryOptimizer()

    dummy_lidar = []
    for x in np.linspace(0.2, 3.0, 30):
        dummy_lidar.append((math.degrees(math.atan2(0.8, x)), math.hypot(x, 0.8) * 1000.0))
    for x in np.linspace(0.2, 3.0, 30):
        dummy_lidar.append(((360.0 + math.degrees(math.atan2(-0.8, x))) % 360.0, math.hypot(x, -0.8) * 1000.0))

    dummy_towers = [
        {"x_m": 0.9, "y_m": 0.2, "dist_m": 0.92, "angle_deg": 12.5, "color": "green"}
    ]

    latencies = []
    for _ in range(100):
        res = planner.plan_optimal_trajectory(
            lidar_points=dummy_lidar,
            towers=dummy_towers,
            base_speed_pwm=110,
            challenge_mode="OBSTACLE_CHALLENGE"
        )
        latencies.append(res['latency_ms'])

    print(f"Optimal Steer Angle: {res['steer_deg']}° (Servo PWM: {res['steer_pwm']})")
    print(f"Optimal Target Speed: {res['target_speed_pwm']} PWM")
    print(f"Optimal Path Points: {len(res['optimal_path'])} waypoints")
    print(f"Candidate Paths: {len(res['candidate_paths'])} sampled")
    print(f"Average Compute Latency: {np.mean(latencies):.2f} ms (Min: {np.min(latencies):.2f} ms, Max: {np.max(latencies):.2f} ms)")
    print("Trajectory optimization test successful!")
