#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sih_bot = get_package_share_directory('sih_bot')
    urdf_file = os.path.join(pkg_sih_bot, 'urdf', 'sih_bot.urdf.xacro')

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='2.70', description='Spawn Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'sih_bot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg, yaw_arg,
        robot_state_pub,
        spawn_entity
    ])
