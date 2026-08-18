import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')
        
        # ROS 2 Subscribers & Publishers
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steering_angle_pub = self.create_publisher(Int32, '/steering_angle', 10)

        # PID Parameters for Wall Following
        self.kp = 0.8
        self.kd = 0.3
        self.target_wall_dist = 0.45 # 45 cm target distance from wall
        self.last_error = 0.0

        self.get_logger().info('RPLIDAR C1 Navigation Node Initialized!')

    def scan_callback(self, msg: LaserScan):
        # Angle ranges for RPLIDAR C1 (360 degrees)
        # Convert index to angle in degrees
        angle_min = math.degrees(msg.angle_min)
        angle_increment = math.degrees(msg.angle_increment)

        front_distances = []
        left_distances = []
        right_distances = []

        for i, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist) or dist <= 0.05:
                continue

            angle = angle_min + (i * angle_increment)
            # Normalize angle to [-180, 180]
            angle = (angle + 180) % 360 - 180

            # Front Sector (-30° to +30°)
            if -30.0 <= angle <= 30.0:
                front_distances.append(dist)
            # Left Sector (60° to 120°)
            elif 60.0 <= angle <= 120.0:
                left_distances.append(dist)
            # Right Sector (-120° to -60°)
            elif -120.0 <= angle <= -60.0:
                right_distances.append(dist)

        min_front = min(front_distances) if front_distances else 5.0
        min_left = min(left_distances) if left_distances else 1.0
        min_right = min(right_distances) if right_distances else 1.0

        # Calculate Steering Correction using PID
        error = min_right - self.target_wall_dist
        derivative = error - self.last_error
        steering_correction = (self.kp * error) + (self.kd * derivative)
        self.last_error = error

        # Map to Servo Angle (90 Center, 62 Right, 118 Left)
        servo_angle = int(90 - (steering_correction * 30.0))
        servo_angle = max(62, min(118, servo_angle))

        # Emergency Stop if obstacle in front < 15cm
        cmd = Twist()
        if min_front < 0.15:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warn(f'Collision Warning! Obstacle at {min_front:.2f}m')
        else:
            cmd.linear.x = 1.16 # Nominal speed 1.16 m/s
            cmd.angular.z = float(servo_angle)

        self.cmd_vel_pub.publish(cmd)

        angle_msg = Int32()
        angle_msg.data = servo_angle
        self.steering_angle_pub.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
