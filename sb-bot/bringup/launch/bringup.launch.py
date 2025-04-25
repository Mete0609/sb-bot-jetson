from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # 路径定义
    imu_node_pkg = 'imu_node'
    motor_controller_pkg = 'motor_controller'
    controller_bridge_pkg = 'controller_bridge'
    lslidar_pkg = 'lslidar_driver'
    # localization_pkg = 'localization'
    description_pkg = 'sb_bot_description'

    lslidar_param = os.path.join(
        get_package_share_directory(lslidar_pkg),
        'params', 'lidar_uart_ros2', 'lsn10p.yaml'
    )

    # urdf_file = os.path.join(
    #     get_package_share_directory('sb_bot_description'),
    #     'urdf',
    #     'sb_robot.urdf.xacro'
    # )

    # 使用 xacro 解析文件并转换为 XML 字符串
    # robot_description_config = xacro.process_file(urdf_file)
    # robot_description = robot_description_config.toxml()
    display_launch_path = os.path.join(
        get_package_share_directory(description_pkg),
        'launch',
        'display.launch.py'
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path)
        ),
        # IMU节点
        Node(
            package=imu_node_pkg,
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        # 电机控制节点
        Node(
            package=motor_controller_pkg,
            executable='motor_controller_node',
            name='motor_controller_node',
            output='screen'
        ),

        # 控制桥接节点
        Node(
            package=controller_bridge_pkg,
            executable='controller_bridge_node',
            name='controller_bridge_node',
            output='screen'
        ),

        # 雷达节点
        Node(
            package=lslidar_pkg,
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            parameters=[lslidar_param]
        ),

        # EKF定位节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('localization'),
                    'launch',
                    'ekf_localization_launch.py'
                )
            )
        ),

        Node(
            package='laser_filters',  # ✅ 启动的是 laser_filters 包中的功能节点
            executable='scan_to_scan_filter_chain',
            name='laser_filter_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('laser_filter_node'),  # ✅ 这是你自己包的名字！
                'config',
                'box_filter.yaml'
            )],
            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ]
        )

    ])
