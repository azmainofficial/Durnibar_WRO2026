import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sulg = get_package_share_directory('sulg_robot')
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    urdf_file = os.path.join(pkg_sulg, 'urdf', 'sulg.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher (TF tree from URDF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # 2. Hardware Bridge Node (ESP32 Motion Controller)
    hw_bridge_node = Node(
        package='sulg_robot',
        executable='sulg_hardware_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_56BA018173-if00',
            'baudrate': 115200,
            'servo_center': 110,
            'publish_tf': True,
            'invert_odom_x': True,
            'invert_odom_y': True,
            'invert_yaw': True
        }]
    )

    # 3. RPLiDAR C1 Scanner Driver (staggered by 2.0s to ensure serial stability)
    lidar_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_sllidar, 'launch', 'sllidar_c1_launch.py')
                ),
                launch_arguments={
                    'channel_type': 'serial',
                    'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f4d6c14f3473ed11b23e6aeefdf7b791-if00-port0',
                    'serial_baudrate': '460800',
                    'frame_id': 'laser_frame',
                    'inverted': 'true',
                    'angle_compensate': 'true',
                    'scan_mode': ''
                }.items()
            )
        ]
    )

    return LaunchDescription([
        rsp_node,
        hw_bridge_node,
        lidar_launch
    ])
