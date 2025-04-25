# from launch import LaunchDescription
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():

#     map_path = os.path.expanduser("~/map/my_map.yaml")

#     config_dir = os.path.join('/home/sb-bot/ros2_ws/robot-driver-main/sb_bot_ws/src/sb-bot/nav_bringup/config/controller_server.yaml')

#     bt_tree_file = os.path.join(
#         get_package_share_directory('nav_bringup'),
#         'behavior_trees',
#         'navigate_through_poses.xml'
#     )
#     # bt_tree_file= os.path.join( "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")
#     # print("BT XML path:", bt_tree_file)

#     return LaunchDescription([

#         # 启动 map_server
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'yaml_filename': map_path}]
#         ),

#         # 启动 AMCL 局部定位器
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[
#                 {'use_sim_time': False},
#                 {'laser_model_type': 'likelihood_field'},
#                 {'min_particles': 500},
#                 {'max_particles': 2000},
#                 {'odom_frame_id': 'odom'},
#                 {'base_frame_id': 'base_footprint'},
#                 {'scan_topic': 'scan_filtered'}
#             ]
#         ),
#         Node(
#             package='nav2_controller',
#             executable='controller_server',
#             name='controller_server',
#             output='screen',
#             parameters=[config_dir, {'robot_base_frame': 'base_footprint'}]
#         ),
#         Node(
#             package='nav2_planner',
#             executable='planner_server',
#             name='planner_server',
#             output='screen',
#             parameters=[{
#                 'base_frame_id': 'base_footprint'},
#                 {'robot_base_frame': 'base_footprint'}]
#         ),
#         Node(
#             package='nav2_bt_navigator',
#             executable='bt_navigator',
#             name='bt_navigator',
#             output='screen',
#             parameters=[{
#                 # 'bt_xml_filename': '/home/sb-bot/ros2_ws/robot-driver-main/sb_bot_ws/src/sb-bot/nav_bringup/behavior_trees/navigate_through_poses.xml'
#                 'bt_xml_filename': bt_tree_file,
#                 'default_nav_to_pose_bt_xml': bt_tree_file,
#                 'default_nav_through_poses_bt_xml': bt_tree_file
#             }]
#         ),
#         # Node(
#         #     package='nav2_recoveries',
#         #     executable='recoveries_server',
#         #     name='recoveries_server',
#         #     output='screen'
#         # ),

#         # lifecycle_manager 管理所有节点
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[
#                 {'use_sim_time': False},
#                 {'autostart': True},
#                 {'node_names': ['map_server',
#                                 'amcl',
#                                 'planner_server',
#                                 'controller_server',
#                                 'bt_navigator'
#                                 # 'recoveries_server'
#                                 ]}
#             ]
#         )
#         # # 自动发布初始位姿
#         # Node(
#         #     package='initialpose_publisher',  # 替换为你的包名
#         #     executable='initial_pose_publisher',
#         #     name='initial_pose_publisher',
#         #     output='screen'
#         # )
#     ])
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    navigation2_dir = get_package_share_directory(
        'sb_bot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(navigation2_dir, 'maps', 'my_map.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])