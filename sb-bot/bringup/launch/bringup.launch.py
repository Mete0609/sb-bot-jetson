from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 路径定义
    imu_node_pkg = 'imu_node'
    motor_controller_pkg = 'motor_controller'
    controller_bridge_pkg = 'controller_bridge'
    lslidar_pkg = 'lslidar_driver'
    localization_pkg = 'localization'
    description_pkg = 'sb_bot_description'

    lslidar_param = os.path.join(
        get_package_share_directory(lslidar_pkg),
        'params', 'lidar_uart_ros2', 'lsn10p.yaml'
    )

    ekf_param = os.path.join(
        get_package_share_directory(localization_pkg),
        'config', 'ekf.yaml'
    )

    return LaunchDescription([
        # 机器人描述发布
        Node(
            package='robot_state_publisher',   
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': False,
                'robot_description': os.popen(
                    f"xacro {os.path.join(get_package_share_directory('sb_bot_description'), 'urdf', 'sb_robot.urdf.xacro')}"
                ).read()
                }
            ]
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
            executable='controller_bridge',
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
    ])
