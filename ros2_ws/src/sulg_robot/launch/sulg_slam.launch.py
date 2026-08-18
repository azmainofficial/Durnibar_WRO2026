import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sulg = get_package_share_directory('sulg_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_sulg, 'config', 'mapper_params_online_async.yaml')
    )

    # 1. Include Sulg Bringup (Hardware Bridge + LiDAR + URDF + Camera)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sulg, 'launch', 'sulg_bringup.launch.py')
        )
    )

    # 2. SLAM Toolbox Online Asynchronous Mapping
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false'
        }.items()
    )

    # 3. Web Dashboard Server (Port 5000)
    web_bridge_node = Node(
        package='sulg_robot',
        executable='sulg_web_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_sulg, 'config', 'mapper_params_online_async.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
        ),
        bringup_launch,
        slam_launch,
        web_bridge_node
    ])
