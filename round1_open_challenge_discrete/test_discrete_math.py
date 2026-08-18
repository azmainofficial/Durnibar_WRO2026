def run_discrete_control(
    discrete_state,
    turn_start_yaw,
    turn_direction,
    esp_yaw,
    front_dist,
    left_dist,
    right_dist,
    cfg
):
    center_pwm = 110
    front_turn_threshold = cfg.get("front_turn_threshold_m", 0.75)
    side_safety_threshold = cfg.get("side_safety_threshold_m", 0.38)
    side_correction = cfg.get("side_correction_pwm", 12)
    target_turn_angle = cfg.get("target_turn_angle_deg", 84.0)
    corner_speed = cfg.get("cornering_speed", 45)
    straight_speed = cfg.get("straight_speed", 55)

    speed = straight_speed
    steer = center_pwm
    action_name = "DRIVE_STRAIGHT_PERFECT"

    # State: DRIVE_STRAIGHT
    if discrete_state == "DRIVE_STRAIGHT":
        if 0.05 < front_dist < front_turn_threshold:
            if left_dist >= right_dist:
                turn_direction = 1  # Left
                steer = 60
                action_name = "DISCRETE_TURN_LEFT"
            else:
                turn_direction = -1 # Right
                steer = 160
                action_name = "DISCRETE_TURN_RIGHT"
            
            turn_start_yaw = esp_yaw
            discrete_state = "TURNING_90"
            speed = corner_speed
        else:
            speed = straight_speed
            if 0.05 < left_dist < side_safety_threshold:
                steer = center_pwm + side_correction
                action_name = "DRIVE_STRAIGHT_ADJ_RIGHT"
            elif 0.05 < right_dist < side_safety_threshold:
                steer = center_pwm - side_correction
                action_name = "DRIVE_STRAIGHT_ADJ_LEFT"
            else:
                steer = center_pwm
                action_name = "DRIVE_STRAIGHT_PERFECT"

    # State: TURNING_90
    elif discrete_state == "TURNING_90":
        speed = corner_speed
        if turn_direction == 1:
            steer = 60
            action_name = "DISCRETE_TURN_LEFT"
        else:
            steer = 160
            action_name = "DISCRETE_TURN_RIGHT"
            
        # Calculate total yaw change
        yaw_diff = esp_yaw - turn_start_yaw
        if yaw_diff > 180.0:
            yaw_diff -= 360.0
        elif yaw_diff < -180.0:
            yaw_diff += 360.0
        yaw_change_mag = abs(yaw_diff)
        
        if yaw_change_mag >= target_turn_angle:
            discrete_state = "DRIVE_STRAIGHT"
            steer = center_pwm
            action_name = "DRIVE_STRAIGHT_PERFECT"
            turn_direction = 0

    return discrete_state, turn_start_yaw, turn_direction, speed, steer, action_name

def test_suite():
    cfg = {
        "front_turn_threshold_m": 0.75,
        "side_safety_threshold_m": 0.38,
        "side_correction_pwm": 12,
        "target_turn_angle_deg": 84.0,
        "cornering_speed": 45,
        "straight_speed": 55
    }

    # Test 1: Normal straight driving (no side walls close)
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "DRIVE_STRAIGHT", 0.0, 0, 0.0, 1.5, 0.8, 0.8, cfg
    )
    print(f"Test 1: State={state}, Steer={steer}, Action={action}")
    assert state == "DRIVE_STRAIGHT"
    assert steer == 110
    assert action == "DRIVE_STRAIGHT_PERFECT"

    # Test 2: Straight driving drifting close to left wall -> steer right (110 + 12 = 122)
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "DRIVE_STRAIGHT", 0.0, 0, 0.0, 1.5, 0.3, 0.8, cfg
    )
    print(f"Test 2: State={state}, Steer={steer}, Action={action}")
    assert state == "DRIVE_STRAIGHT"
    assert steer == 122
    assert action == "DRIVE_STRAIGHT_ADJ_RIGHT"

    # Test 3: Straight driving drifting close to right wall -> steer left (110 - 12 = 98)
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "DRIVE_STRAIGHT", 0.0, 0, 0.0, 1.5, 0.8, 0.3, cfg
    )
    print(f"Test 3: State={state}, Steer={steer}, Action={action}")
    assert state == "DRIVE_STRAIGHT"
    assert steer == 98
    assert action == "DRIVE_STRAIGHT_ADJ_LEFT"

    # Test 4: Corner ahead (front_dist = 0.6m). Left has more room -> Transition to TURNING_90 Left
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "DRIVE_STRAIGHT", 0.0, 0, 45.0, 0.6, 1.2, 0.8, cfg
    )
    print(f"Test 4: State={state}, StartYaw={start_yaw}, TurnDir={turn_dir}, Steer={steer}, Action={action}")
    assert state == "TURNING_90"
    assert turn_dir == 1
    assert start_yaw == 45.0
    assert steer == 60
    assert action == "DISCRETE_TURN_LEFT"

    # Test 5: Mid-turn (TURNING_90 Left, starting at 45.0, current yaw is 90.0 -> diff is 45.0 deg). Should keep turning.
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "TURNING_90", 45.0, 1, 90.0, 1.5, 1.2, 0.8, cfg
    )
    print(f"Test 5: State={state}, Steer={steer}, Action={action}")
    assert state == "TURNING_90"
    assert steer == 60

    # Test 6: Turn completion (yaw is 130.0 -> diff is 85.0 deg >= 84.0). Should transition back to DRIVE_STRAIGHT and straighten wheels.
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "TURNING_90", 45.0, 1, 130.0, 1.5, 1.2, 0.8, cfg
    )
    print(f"Test 6: State={state}, Steer={steer}, Action={action}")
    assert state == "DRIVE_STRAIGHT"
    assert steer == 110
    assert turn_dir == 0
    assert action == "DRIVE_STRAIGHT_PERFECT"

    # Test 7: Angle wrap-around turn completion (start yaw 350.0, current yaw 75.0 -> change 85.0 deg)
    state, start_yaw, turn_dir, speed, steer, action = run_discrete_control(
        "TURNING_90", 350.0, 1, 75.0, 1.5, 1.2, 0.8, cfg
    )
    print(f"Test 7: State={state}, Steer={steer}, Action={action}")
    assert state == "DRIVE_STRAIGHT"
    assert steer == 110
    assert turn_dir == 0

    print("[SUCCESS] Discrete state machine transition and yaw math tests passed!")

if __name__ == '__main__':
    test_suite()
