import math

def calculate_steering(esp_x, esp_y, esp_yaw, target_wp, center_pwm=110, kp_steer=1.2):
    dx = target_wp[0] - esp_x
    dy = target_wp[1] - esp_y
    dist_to_wp = math.sqrt(dx**2 + dy**2)
    
    target_yaw_deg = math.degrees(math.atan2(dy, dx)) % 360.0
    heading_error = target_yaw_deg - esp_yaw
    
    if heading_error > 180.0:
        heading_error -= 360.0
    elif heading_error < -180.0:
        heading_error += 360.0
        
    steer = max(60, min(160, center_pwm - int(round(kp_steer * heading_error))))
    return dist_to_wp, target_yaw_deg, heading_error, steer

def run_tests():
    # Test 1: Robot at (0,0), facing 0, target at (2000, 0) -> Should drive straight
    dist, target_yaw, error, steer = calculate_steering(0, 0, 0, (2000, 0))
    print(f"Test 1: Dist={dist:.1f}, TargetYaw={target_yaw:.1f}, Error={error:.1f}, Steer={steer}")
    assert abs(error) < 1e-3
    assert steer == 110
    
    # Test 2: Robot at (0,0), facing 0, target at (0, 1000) -> Should steer left (error = +90, steer < 110)
    dist, target_yaw, error, steer = calculate_steering(0, 0, 0, (0, 1000))
    print(f"Test 2: Dist={dist:.1f}, TargetYaw={target_yaw:.1f}, Error={error:.1f}, Steer={steer}")
    assert error == 90.0
    assert steer == 60 # Clamped from 110 - 1.2*90 = 2
    
    # Test 3: Robot at (0,0), facing 0, target at (0, -1000) -> Should steer right (error = -90, steer > 110)
    dist, target_yaw, error, steer = calculate_steering(0, 0, 0, (0, -1000))
    print(f"Test 3: Dist={dist:.1f}, TargetYaw={target_yaw:.1f}, Error={error:.1f}, Steer={steer}")
    assert error == -90.0
    assert steer == 160 # Clamped from 110 - 1.2*(-90) = 218
    
    # Test 4: Angle wrapping. Robot facing 350 deg, target at (0, 1000) (target_yaw = 90 deg)
    # Error should be +100 deg (turn left)
    dist, target_yaw, error, steer = calculate_steering(0, 0, 350, (0, 1000))
    print(f"Test 4: Dist={dist:.1f}, TargetYaw={target_yaw:.1f}, Error={error:.1f}, Steer={steer}")
    assert error == 100.0
    assert steer == 60
    
    # Test 5: Angle wrapping. Robot facing 10 deg, target at (0, -1000) (target_yaw = 270 deg)
    # Error should be -100 deg (turn right)
    dist, target_yaw, error, steer = calculate_steering(0, 0, 10, (0, -1000))
    print(f"Test 5: Dist={dist:.1f}, TargetYaw={target_yaw:.1f}, Error={error:.1f}, Steer={steer}")
    assert error == -100.0
    assert steer == 160

    print("[SUCCESS] All waypoint navigation math tests passed!")

if __name__ == '__main__':
    run_tests()
