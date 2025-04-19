from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 获取 xacro 文件路径
    urdf_file = os.path.join(
        get_package_share_directory('sb_bot_description'),
        'urdf',
        'sb_robot.urdf.xacro'
    )

    # 使用 xacro 解析文件并转换为 XML 字符串
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('sb_bot_description'),
                'rviz',
                'robot.rviz'
            )]
        )
    ])
