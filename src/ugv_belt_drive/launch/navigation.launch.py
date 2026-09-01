import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_belt_drive')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', '/ugv/cmd_vel'),
        ('/odom', '/ugv/odom')
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True
        ),
        allow_substs=True
    )

    load_nodes = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/ugv/cmd_vel'),
            SetRemap(src='/odom', dst='/ugv/odom'),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/ugv/cmd_vel')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/ugv/cmd_vel')]
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=False,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/ugv/cmd_vel'), ('cmd_vel_smoothed', '/ugv/cmd_vel')]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]
            )
        ]
    )

    use_static_map_tf = LaunchConfiguration('use_static_map_tf', default='false')

    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_static_map_tf)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_static_map_tf', default_value='false', description='Publish static map->odom TF when running without SLAM'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
                              description='Full path to the ROS2 parameters file to use'),
        load_nodes,
        static_map_tf
    ])
