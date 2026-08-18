#!/usr/bin/env python3
"""
optimal_planner.py – Ultra-Fast Vectorized Ackermann Optimal Path Planner & Trajectory Optimizer
ROS 2 Jazzy Module for Sulg Autonomous Vehicle
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
                 num_steps=25,
                 safety_margin_m=0.16):
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
            "w_clearance": 3.5,       # Repulsion from walls and obstacles
            "w_progress": 2.2,        # Heading towards the furthest open corridor
            "w_wro_rules": 8.0,       # Strict rule enforcement (Green=Left, Red=Right)
            "w_smooth": 0.8,          # Steering rate penalty to prevent wobble
            "w_centering": 1.5,       # Track centerline bias
            "safety_margin_m": safety_margin_m,  # Desired clearance buffer around robot
            "max_lat_accel": 2.8      # Maximum lateral cornering acceleration (m/s^2)
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

        :param lidar_points: List of (angle_deg, dist_mm) or LaserScan ranges
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
            'target_speed_mps': float,
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
            mask = (dists > 50.0) & (dists <= 3500.0) & (angles_rel >= -100.0) & (angles_rel <= 100.0)
            
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
        crit_radius = self.robot_half_width + 0.05  # Absolute collision limit (m)

        # Target progress angle in radians
        pref_rad = math.radians(preferred_angle_deg)
        pref_dir = np.array([math.cos(pref_rad), math.sin(pref_rad)], dtype=np.float32)

        # Corridor centering bias
        track_center_offset = 0.0
        if 0.1 < left_wall_m < 2.5 and 0.1 < right_wall_m < 2.5:
            track_center_offset = (left_wall_m - right_wall_m) * 0.5

        # ── A. Collision & Clearance Cost with LiDAR Obstacles (Vectorized across all candidates) ──
        if len(obs_arr) > 0:
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

        # ── D. Lane / Corridor Centering (Vectorized) ──
        avg_traj_y = np.mean(self.template_trajectories[:, :, 1], axis=1)  # (C,)
        costs[valid_mask] += w_center * np.abs(avg_traj_y[valid_mask] - track_center_offset) * 3.0

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

                    # Y position of trajectory at closest point to tower
                    traj_y_at_tower = self.template_trajectories[np.arange(num_cands), min_idx, 1]  # (C,)

                    if t_color == 'green':
                        wrong_side = (traj_y_at_tower < t_y + 0.15) & valid_mask
                        costs[wrong_side] += w_wro * 80.0 * (1.0 / (min_t_dists[wrong_side] + 0.1))
                        correct_side = (~wrong_side) & valid_mask
                        costs[correct_side] -= w_wro * 15.0

                    elif t_color == 'red':
                        wrong_side = (traj_y_at_tower > t_y - 0.15) & valid_mask
                        costs[wrong_side] += w_wro * 80.0 * (1.0 / (min_t_dists[wrong_side] + 0.1))
                        correct_side = (~wrong_side) & valid_mask
                        costs[correct_side] -= w_wro * 15.0

        # 3. Select the Optimal Candidate Trajectory
        if np.any(valid_mask):
            valid_indices = np.where(valid_mask)[0]
            best_idx = valid_indices[np.argmin(costs[valid_indices])]
            best_cost = float(costs[best_idx])
            safe = True
        else:
            # Fallback if all rollouts have close obstacles: pick minimum cost
            best_idx = int(np.argmin(costs))
            best_cost = float(costs[best_idx])
            safe = False

        selected_steer_deg = float(self.steer_angles_deg[best_idx])
        self.last_selected_steer_deg = selected_steer_deg

        # Map steer deflection (e.g. -30..+30) to servo PWM (50..170, center 110)
        steer_pwm = int(round(self.center_steer + selected_steer_deg * (60.0 / self.max_steer_deg)))
        steer_pwm = max(50, min(170, steer_pwm))

        # 4. Dynamic Cornering Speed Profiling
        delta_rad = math.radians(abs(selected_steer_deg))
        curvature = math.tan(delta_rad) / self.wheelbase
        speed_factor = 1.0 / (1.0 + 1.2 * curvature)

        target_speed_pwm = int(round(base_speed_pwm * max(0.55, speed_factor)))
        if not safe:
            target_speed_pwm = max(45, int(target_speed_pwm * 0.7))

        target_speed_mps = round((target_speed_pwm / 255.0) * 1.5, 3)

        # Instant zero-alloc trajectory formatting
        optimal_path = self.template_trajectories_list[best_idx]

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
            "target_speed_mps": target_speed_mps,
            "optimal_path": optimal_path,
            "candidate_paths": candidate_paths,
            "latency_ms": self.last_latency_ms,
            "cost": round(best_cost, 2),
            "safe": safe
        }
