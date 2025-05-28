from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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

    # 使用 xacro 解析并转换为 XML 字符串
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        # 声明 use_sim_time 和 with_clock 参数
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('with_clock', default_value='false'),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description
            }]
        ),

        # 可选 clock_publisher（用于模拟时间）
        GroupAction([
            Node(
                package='clock_tools',
                executable='clock_publisher',
                name='clock_publisher',
                output='screen'
            )
        ], condition=IfCondition(LaunchConfiguration('with_clock')))
    ])
