import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sulg = get_package_share_directory('sulg_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_sulg, 'config', 'nav2_params.yaml')
    )
    map_yaml_file = LaunchConfiguration('map', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    # 1. Include Sulg Bringup (Hardware Bridge + LiDAR + URDF + Camera)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sulg, 'launch', 'sulg_bringup.launch.py')
        )
    )

    # 2. Navigation 2 Stack (planner, controller, bt_navigator, costmaps)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart
        }.items()
    )

    # 3. HTML5 Web Dashboard Bridge
    web_bridge_node = Node(
        package='sulg_robot',
        executable='sulg_web_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_sulg, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Nav2 nodes'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        bringup_launch,
        nav2_launch,
        web_bridge_node
    ])
