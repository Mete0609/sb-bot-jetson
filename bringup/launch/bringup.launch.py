import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 参数定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_clock = LaunchConfiguration('with_clock', default='false')

    # 路径定义
    imu_node_pkg = 'imu_node'
    motor_controller_pkg = 'motor_controller'
    controller_bridge_pkg = 'controller_bridge'
    lslidar_pkg = 'lslidar_driver'
    description_pkg = 'sb_bot_description'

    lslidar_param = os.path.join(
        get_package_share_directory(lslidar_pkg),
        'params', 'lidar_uart_ros2', 'lsn10p.yaml'
    )

    display_launch_path = os.path.join(
        get_package_share_directory(description_pkg),
        'launch',
        'display.launch.py'
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([

        # === 参数声明 ===
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('with_clock', default_value='false', description='Publish /clock topic if true'),

        # === 1. 仿真时钟源（最先） ===
        GroupAction([
            Node(
                package='clock_tools',
                executable='clock_publisher',
                name='clock_publisher',
                output='screen'
            )
        ], condition=IfCondition(with_clock)),

        # === 2. 发布 robot_state_publisher / 机器人模型 ===
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(display_launch_path)
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'with_clock': with_clock
            }.items()
        ),

        # === 3. 启动 IMU + 雷达 + 雷达滤波器 ===
        Node(
            package=imu_node_pkg,
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package=lslidar_pkg,
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[lslidar_param, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('laser_filter_node'),
                'config', 'box_filter.yaml')],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ]
        ),

        # === 4. 启动 EKF（用于广播 odom->base_link） ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('localization'),
                    'launch',
                    'ekf_localization_launch.py'
                )
            )
        ),

        # === 5. 启动 motor controller 和控制桥 ===
        Node(
            package=motor_controller_pkg,
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package=controller_bridge_pkg,
            executable='controller_bridge_node',
            name='controller_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # === 6. 启动 Navigation2（依赖前面的 TF、传感器） ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sb_bot_navigation2'),
                    'launch',
                    'navigation2.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # === 7. nav_bridge（辅助节点） ===
        Node(
            package='nav_bridge',
            executable='nav_bridge_node',
            name='nav_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        #位姿调整节点（开启监听）
        Node(
            package='target_pose_controller',
            executable='pose_controller_node',
            name='pose_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        #机械臂启动节点
        Node(
            package='joint_controller',
            executable='joint_controller_node',
            name='joint_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
