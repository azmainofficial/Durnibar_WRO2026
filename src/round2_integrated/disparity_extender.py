#!/usr/bin/env python3
"""
disparity_extender.py – Disparity Extender Algorithm & Fused LiDAR+CV Tower Detector
For WRO Autonomous Navigation on Raspberry Pi 5
"""

import math
import numpy as np

class DisparityExtender:
    def __init__(self, robot_width_m=0.22, disparity_threshold_m=0.3, max_range_m=3.5):
        self.robot_width_m = robot_width_m
        self.disparity_threshold_m = disparity_threshold_m
        self.max_range_m = max_range_m

    def process_scan(self, points):
        """
        points: list of (angle_deg, dist_mm)
        Returns: target_angle_deg, best_dist_m, extended_ranges_dict
        """
        if not points:
            return 0.0, 0.0, {}

        # Interpolate points onto 360-degree array
        ranges = np.full(360, self.max_range_m, dtype=np.float32)
        for a_deg, d_mm in points:
            idx = int(round(a_deg)) % 360
            d_m = d_mm / 1000.0
            if 0.16 <= d_m <= self.max_range_m:
                ranges[idx] = min(ranges[idx], d_m)

        extended = np.copy(ranges)

        # Detect disparities & extend safety bubbles
        for i in range(360):
            d1 = ranges[i]
            d2 = ranges[(i + 1) % 360]
            diff = abs(d1 - d2)

            if diff > self.disparity_threshold_m:
                closer_d = min(d1, d2)
                if closer_d > 0.05:
                    # Calculate angle bubble width needed
                    angle_bubble_rad = math.atan2(self.robot_width_m / 2.0, closer_d)
                    angle_bubble_deg = int(math.ceil(math.degrees(angle_bubble_rad)))

                    start_idx = i if d1 < d2 else (i + 1)
                    step = 1 if d1 < d2 else -1

                    for b in range(angle_bubble_deg):
                        target_idx = (start_idx + step * b) % 360
                        extended[target_idx] = min(extended[target_idx], closer_d)

        # Focus front sector navigation (-75 deg to +75 deg)
        front_indices = []
        for a in range(-75, 76):
            idx = a % 360
            front_indices.append((a, extended[idx]))

        # Find best gap (furthest clear path in front sector)
        front_indices.sort(key=lambda x: x[1], reverse=True)
        best_angle, best_dist = front_indices[0] if front_indices else (0, 1.0)

        # Build dictionary for output
        extended_dict = {a % 360: float(extended[a % 360]) for a in range(360)}
        return float(best_angle), float(best_dist), extended_dict

class LidarCvTowerFusion:
    def __init__(self, cluster_tolerance_m=0.15, min_cluster_points=3):
        self.cluster_tolerance_m = cluster_tolerance_m
        self.min_cluster_points = min_cluster_points

    def cluster_lidar_points(self, points):
        """
        Group 2D Cartesian LiDAR points into tower clusters
        """
        if not points:
            return []

        # Convert polar to Cartesian (x_m, y_m)
        cartesian = []
        for a_deg, d_mm in points:
            d_m = d_mm / 1000.0
            if d_m <= 0.16 or d_m > 3.5:
                continue
            rad = math.radians(a_deg)
            # Standard robot frame: x forward, y left
            x = d_m * math.cos(rad)
            y = d_m * math.sin(rad)
            cartesian.append((x, y, a_deg, d_m))

        if not cartesian:
            return []

        # BFS Euclidean Distance Clustering
        clusters = []
        visited = [False] * len(cartesian)

        for i in range(len(cartesian)):
            if visited[i]:
                continue
            
            current_cluster = []
            queue = [i]
            visited[i] = True
            
            head_idx = 0
            while head_idx < len(queue):
                curr_idx = queue[head_idx]
                head_idx += 1
                curr_pt = cartesian[curr_idx]
                current_cluster.append(curr_pt)
                
                for j in range(len(cartesian)):
                    if not visited[j]:
                        dx = curr_pt[0] - cartesian[j][0]
                        dy = curr_pt[1] - cartesian[j][1]
                        if math.hypot(dx, dy) <= self.cluster_tolerance_m:
                            visited[j] = True
                            queue.append(j)

            if len(current_cluster) >= self.min_cluster_points:
                avg_x = sum(pt[0] for pt in current_cluster) / len(current_cluster)
                avg_y = sum(pt[1] for pt in current_cluster) / len(current_cluster)
                avg_dist = math.hypot(avg_x, avg_y)
                avg_angle = math.degrees(math.atan2(avg_y, avg_x)) % 360.0

                clusters.append({
                    "x_m": round(avg_x, 3),
                    "y_m": round(avg_y, 3),
                    "dist_m": round(avg_dist, 3),
                    "angle_deg": round(avg_angle, 1),
                    "num_points": len(current_cluster),
                    "color": "unknown"
                })

        return clusters

    def fuse_towers(self, lidar_clusters, cv_objects, fov_deg=109.0, image_width=640):
        """
        Match camera-detected colored objects with LiDAR 2D spatial clusters
        """
        fused = [dict(c) for c in lidar_clusters]

        for obj in cv_objects:
            cx = obj['cx']
            color = obj.get('color', obj.get('class', 'unknown'))
            if color == 'unknown':
                continue
            
            # Map cx (0..640) to camera polar angle offset relative to center
            # In camera: cx < 320 is LEFT (+angle in LiDAR), cx > 320 is RIGHT (-angle in LiDAR)
            angle_offset = (((image_width / 2.0) - cx) / (image_width / 2.0)) * (fov_deg / 2.0)

            # Find closest LiDAR cluster by angle & distance
            best_match = None
            min_angle_diff = float('inf')

            for cluster in fused:
                # Convert cluster angle to -180..180 format for comparison (+ is left, - is right)
                c_angle = cluster['angle_deg']
                if c_angle > 180:
                    c_angle -= 360.0

                diff = abs(c_angle - angle_offset)
                if diff < min_angle_diff and diff <= 40.0:  # Expanded 40-degree search window
                    min_angle_diff = diff
                    best_match = cluster

            if best_match:
                best_match['color'] = color
                best_match['cv_cx'] = cx
                best_match['cv_cy'] = obj['cy']
            else:
                # Fallback: create virtual tower from camera pinhole estimate if no LiDAR cluster matched
                cam_dist_m = obj.get('dist_m', 0.8)
                rad = math.radians(angle_offset)
                vx = cam_dist_m * math.cos(rad)
                vy = cam_dist_m * math.sin(rad)
                fused.append({
                    "x_m": round(vx, 3),
                    "y_m": round(vy, 3),
                    "dist_m": round(cam_dist_m, 3),
                    "angle_deg": round(angle_offset % 360.0, 1),
                    "num_points": 1,
                    "color": color,
                    "cv_cx": cx,
                    "cv_cy": obj['cy']
                })

        return fused

# Quick Test
if __name__ == '__main__':
    de = DisparityExtender()
    dummy_points = [(i, 1000 if i < 180 else 2500) for i in range(0, 360, 2)]
    target_a, target_d, ext = de.process_scan(dummy_points)
    print(f"[TEST] Disparity Extender Target Angle: {target_a} deg, Distance: {target_d} m")
