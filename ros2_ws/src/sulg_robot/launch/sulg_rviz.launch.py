import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sulg = get_package_share_directory('sulg_robot')
    rviz_config_file = os.path.join(pkg_sulg, 'rviz', 'sulg_nav.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([rviz_node])
