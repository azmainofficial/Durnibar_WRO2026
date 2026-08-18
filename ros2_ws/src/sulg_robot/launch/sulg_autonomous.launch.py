import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sulg = get_package_share_directory('sulg_robot')

    # Include Bringup (Hardware, Camera, LiDAR, URDF)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sulg, 'launch', 'sulg_bringup.launch.py')
        )
    )

    # Autonomous Optimal Trajectory Planner Node
    nav_node = Node(
        package='sulg_robot',
        executable='sulg_autonomous_nav',
        output='screen',
        parameters=[{
            'challenge_mode': 'OPEN_CHALLENGE',
            'nominal_speed_pwm': 120,
            'max_speed_mps': 1.5,
            'wheelbase_m': 0.20,
            'safety_margin_m': 0.16
        }]
    )

    # HTML5 Web Dashboard Bridge Node
    web_bridge_node = Node(
        package='sulg_robot',
        executable='sulg_web_bridge',
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        nav_node,
        web_bridge_node
    ])
