import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(command, *args, **kwargs):
    mode = command.launch_configurations['mode']

    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml')

    if mode == 'real':
        planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
        rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanning.rviz')
        recovery_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'recovery.yaml')
        cmd_vel_remap = '/cmd_vel'
        use_sim_time = False

    else:
        planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_planner_server.yaml')
        controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_controller.yaml')
        rviz_config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'sim_pathplanning.rviz')
        recovery_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'sim_recovery.yaml')
        cmd_vel_remap = '/diffbot_base_controller/cmd_vel_unstamped'
        use_sim_time = True

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', cmd_vel_remap)]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server', 'behavior_server',
                                        'bt_navigator']}]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_nav_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_dir]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='real',
            description='Simulation or real mode.'),
        OpaqueFunction(function=launch_setup)
    ])