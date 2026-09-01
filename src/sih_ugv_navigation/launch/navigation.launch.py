#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sih_ugv_gazebo = get_package_share_directory('sih_ugv_gazebo')
    pkg_sih_ugv_navigation = get_package_share_directory('sih_ugv_navigation')

    # Arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2 Dashboard')
    ekf_arg = DeclareLaunchArgument('ekf', default_value='true', description='Enable EKF Filter')
    nav_arg = DeclareLaunchArgument('auto_nav', default_value='true', description='Enable Autonomous Navigator')

    rviz = LaunchConfiguration('rviz')
    ekf = LaunchConfiguration('ekf')
    auto_nav = LaunchConfiguration('auto_nav')

    ekf_config_file = join(pkg_sih_ugv_navigation, 'config', 'ekf.yaml')
    rviz_config_file = join(pkg_sih_ugv_navigation, 'config', 'sih_dashboard.rviz')

    # 1. Gazebo Outdoor Simulation Bringup
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_sih_ugv_gazebo, 'launch', 'sim_outdoor.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # 2. Robot Localization EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        condition=IfCondition(ekf)
    )

    # 3. Perception AI: Path Segmentation & Traversability Node
    path_segmentation_node = Node(
        package='sih_ugv_perception',
        executable='path_segmentation_node',
        name='path_segmentation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Perception AI: Visual Beacon & AprilTag Detector
    visual_beacon_detector = Node(
        package='sih_ugv_perception',
        executable='visual_beacon_detector',
        name='visual_beacon_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. Autonomous Mission Navigator & Dynamic Collision Avoidance
    autonomous_navigator = Node(
        package='sih_ugv_navigation',
        executable='autonomous_navigator',
        name='autonomous_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(auto_nav)
    )

    # 6. RViz2 SIH Dashboard Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_dashboard',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        rviz_arg,
        ekf_arg,
        nav_arg,
        sim_bringup,
        ekf_node,
        path_segmentation_node,
        visual_beacon_detector,
        autonomous_navigator,
        rviz_node,
    ])
