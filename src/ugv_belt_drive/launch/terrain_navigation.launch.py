#!/usr/bin/env python3
"""
Master Integrated Launch File: Terrain Navigation Pipeline
Launches:
1. Gazebo Harmonic 3D World with real elevation & satellite texture
2. Custom 4-Sprocket Belt-Drive UGV spawned at Point A (start coordinates)
3. Sensor Bridges (LiDAR, Camera, IMU, Odometry, cmd_vel)
4. SLAM Toolbox (Online mapping of unknown terrain)
5. Nav2 Autonomous Navigation Stack (Costmaps, Global/Local Planners, Recoveries)
6. RViz2 3D Perception & Path Visualizer
7. Autonomous Goal Dispatcher targeting Point B
"""

import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ugv_belt_drive')

    world_arg = LaunchConfiguration('world').perform(context)
    start_x_arg = LaunchConfiguration('start_x').perform(context)
    start_y_arg = LaunchConfiguration('start_y').perform(context)
    start_z_arg = LaunchConfiguration('start_z').perform(context)
    goal_x_arg = LaunchConfiguration('goal_x').perform(context)
    goal_y_arg = LaunchConfiguration('goal_y').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    autostart_mission = LaunchConfiguration('autostart_mission').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve world file path
    if not os.path.isabs(world_arg) or not os.path.isfile(world_arg):
        candidate = os.path.join(pkg_share, 'worlds', world_arg)
        if os.path.isfile(candidate):
            world_file = candidate
        else:
            world_file = world_arg
    else:
        world_file = world_arg

    world_dir = os.path.dirname(os.path.abspath(world_file)) if os.path.isfile(world_file) else pkg_share

    # Auto-load Point A and Point B from terrain_metadata.json if available
    goal_x = float(goal_x_arg)
    goal_y = float(goal_y_arg)
    start_x = float(start_x_arg)
    start_y = float(start_y_arg)
    start_z = float(start_z_arg)

    meta_path = os.path.join(world_dir, 'terrain_metadata.json')
    if os.path.isfile(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
            pt_a = meta.get('point_a', {})
            pt_b = meta.get('point_b', {})
            if 'gazebo_x' in pt_a and start_x_arg == '0.0':
                start_x = float(pt_a.get('gazebo_x', 0.0))
                start_y = float(pt_a.get('gazebo_y', 0.0))
            if 'gazebo_x' in pt_b and goal_x_arg == '5.0':
                goal_x = float(pt_b.get('gazebo_x', 5.0))
                goal_y = float(pt_b.get('gazebo_y', 0.0))
        except Exception as e:
            print(f"[WARN] Error reading terrain_metadata.json: {e}")

    # 1. Spawn Robot & Gazebo Simulation
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'world': world_file,
            'x': str(start_x),
            'y': str(start_y),
            'z': str(start_z),
            'headless': headless,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. SLAM Toolbox & RViz2 (delayed slightly to ensure Gazebo clock & bridge are up)
    slam_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': rviz
                }.items()
            )
        ]
    )

    # 3. Nav2 Navigation Stack (delayed slightly for SLAM map availability)
    nav2_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 4. Autonomous Mission Dispatcher targeting Point B
    mission_actions = []
    if autostart_mission:
        mission_cmd = [
            'python3',
            os.path.join(pkg_share, 'scripts', 'ugv_autonomous_mission.py'),
            '--goal_x', str(goal_x),
            '--goal_y', str(goal_y),
            '--goal_yaw', '0.0',
            '--timeout', '90.0'
        ]
        mission_dispatcher = TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=mission_cmd,
                    output='screen'
                )
            ]
        )
        mission_actions.append(mission_dispatcher)

    return [
        spawn_robot_launch,
        slam_launch,
        nav2_launch,
        *mission_actions
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_belt_drive')
    default_world = os.path.join(pkg_share, 'worlds', 'terrain_world.world')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Gazebo world file path'),
        DeclareLaunchArgument('start_x', default_value='0.0', description='Point A (Start) X coordinate'),
        DeclareLaunchArgument('start_y', default_value='0.0', description='Point A (Start) Y coordinate'),
        DeclareLaunchArgument('start_z', default_value='2.0', description='Point A (Start) safe spawn height'),
        DeclareLaunchArgument('goal_x', default_value='5.0', description='Point B (Goal) X coordinate'),
        DeclareLaunchArgument('goal_y', default_value='0.0', description='Point B (Goal) Y coordinate'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo simulation headless'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2 visualizer'),
        DeclareLaunchArgument('autostart_mission', default_value='true', description='Auto-dispatch Point B goal'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        OpaqueFunction(function=launch_setup)
    ])
