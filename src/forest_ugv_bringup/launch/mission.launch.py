#!/usr/bin/env python3

import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    scenario_str = LaunchConfiguration('scenario').perform(context)
    gps_enabled_str = LaunchConfiguration('gps_enabled').perform(context)
    rviz_str = LaunchConfiguration('rviz').perform(context)

    pkg_gazebo = get_package_share_directory('forest_ugv_gazebo')
    pkg_slam = get_package_share_directory('forest_visual_slam')
    pkg_vis = get_package_share_directory('forest_visualization')

    # Resolve world path
    if not scenario_str.endswith('.sdf'):
        world_path = join(pkg_gazebo, 'worlds', f'{scenario_str}.sdf')
    else:
        world_path = join(pkg_gazebo, 'worlds', scenario_str)

    if not os.path.exists(world_path):
        world_path = join(pkg_gazebo, 'worlds', 'forest_world.sdf')

    ekf_config_file = join(pkg_slam, 'config', 'ekf_visual_inertial.yaml')
    rviz_config_file = join(pkg_vis, 'config', 'forest_dashboard.rviz')

    # 1. Gazebo Simulation & ROS Bridge
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_gazebo, 'launch', 'sim_forest.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'rviz': 'false'
        }.items()
    )

    # 2. Sensor Fusion EKF (VIO)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}]
    )

    # 3. AI Perception Node
    perception_node = Node(
        package='forest_perception',
        executable='perception_node',
        name='forest_perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. GPS-Free Visual Odometry & SLAM Node
    visual_slam_node = Node(
        package='forest_visual_slam',
        executable='visual_odometry_node',
        name='forest_visual_slam_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Traversability Costmap Mapping Node
    mapping_node = Node(
        package='forest_mapping',
        executable='traversability_mapping_node',
        name='forest_mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Global A* Path Planner
    global_planner_node = Node(
        package='forest_planner',
        executable='global_path_planner',
        name='forest_global_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. Reactive Local Planner & Dynamic Obstacle Avoidance
    local_planner_node = Node(
        package='forest_planner',
        executable='reactive_local_planner',
        name='forest_local_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Mission Performance & ATE Evaluator
    eval_node = Node(
        package='forest_evaluation',
        executable='mission_evaluator',
        name='forest_mission_evaluator_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'gps_enabled': (gps_enabled_str.lower() == 'true'),
            'scenario_name': scenario_str,
            'output_file': '/home/joshika/Desktop/SIH/evaluation_results.json'
        }]
    )

    # 9. HERCULES Mission Control Web Dashboard (port 8080)
    dashboard_node = Node(
        package='forest_dashboard',
        executable='dashboard_server',
        name='hercules_dashboard_server',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. Google Gemini Multimodal VLA Brain Node
    gemini_brain_node = Node(
        package='forest_gemini_brain',
        executable='gemini_brain_node',
        name='forest_gemini_brain_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_gemini'))
    )

    # 11. RViz2 Dashboard (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_forest_dashboard',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        sim_launch,
        ekf_node,
        perception_node,
        visual_slam_node,
        mapping_node,
        global_planner_node,
        local_planner_node,
        eval_node,
        dashboard_node,
        gemini_brain_node,
        rviz_node,
    ]


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='forest_world',
        description='Forest scenario (forest_world, forest_extreme_hardcore, forest_open_trail, forest_rocky, forest_fallen_tree, forest_ditch_slope, forest_dynamic_obstacle)'
    )
    gps_enabled_arg = DeclareLaunchArgument(
        'gps_enabled',
        default_value='false',
        description='Whether GPS is enabled (false = GPS-Denied Vision-Only)'
    )
    enable_gemini_arg = DeclareLaunchArgument(
        'enable_gemini',
        default_value='true',
        description='Enable Google Gemini Multimodal Brain'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 Visualization Dashboard'
    )

    return LaunchDescription([
        scenario_arg,
        gps_enabled_arg,
        enable_gemini_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
