"""
Master Bringup Launch File for Autonomous UGV Navigation Stack.
Launches EKF Estimator, Semantic Costmap, Global A* Planner, MPPI Local Controller, and Safety Watchdog.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_ugv_nav')

    costmap_config = os.path.join(pkg_dir, 'config', 'costmap_params.yaml')
    mppi_config = os.path.join(pkg_dir, 'config', 'mppi_params.yaml')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/oak/points',
        description='Point cloud topic from OAK-D or camera pipeline'
    )

    # 1. EKF State Estimator Node
    ekf_node = Node(
        package='autonomous_ugv_nav',
        executable='ekf_state_estimator_node',
        name='ekf_state_estimator_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    # 2. Semantic Costmap & Traversability Node
    costmap_node = Node(
        package='autonomous_ugv_nav',
        executable='semantic_costmap_node',
        name='semantic_costmap_node',
        output='screen',
        parameters=[costmap_config, {
            'use_sim_time': use_sim_time,
            'pointcloud_topic': pointcloud_topic
        }]
    )

    # 3. Global Planner Node (Weighted A*)
    global_planner_node = Node(
        package='autonomous_ugv_nav',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. MPPI Local Controller Node
    mppi_node = Node(
        package='autonomous_ugv_nav',
        executable='mppi_controller_node',
        name='mppi_controller_node',
        output='screen',
        parameters=[mppi_config, {'use_sim_time': use_sim_time}]
    )

    # 5. Safety Watchdog Monitor Node
    safety_node = Node(
        package='autonomous_ugv_nav',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_pointcloud_topic,
        ekf_node,
        costmap_node,
        global_planner_node,
        mppi_node,
        safety_node,
    ])
