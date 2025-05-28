from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('laser_filter_node'),
        'config',
        'box_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[config_file],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ],
            output='screen'
        )
    ])
