import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. RPLIDAR C1 Driver Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_c1',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB_LIDAR',
                'serial_baudrate': 460800, # RPLIDAR C1 default baudrate
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # 2. Fifine K420 Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='fifine_k420_camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [1920, 1080],
                'camera_frame_id': 'camera_frame'
            }]
        ),

        # 3. Vision Color Detection Node
        Node(
            package='durnibar_vision',
            executable='color_pillar_node',
            name='color_pillar_node',
            output='screen'
        ),

        # 4. LiDAR Navigation & Obstacle Node
        Node(
            package='durnibar_nav',
            executable='lidar_obstacle_node',
            name='lidar_obstacle_node',
            output='screen'
        ),

        # 5. High-Level FSM State Machine Node
        Node(
            package='durnibar_fsm',
            executable='fsm_node',
            name='fsm_node',
            output='screen'
        )
    ])
