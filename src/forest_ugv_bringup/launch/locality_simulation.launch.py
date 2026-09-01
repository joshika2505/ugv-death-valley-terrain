#!/usr/bin/env python3
"""
Master Launch File: GPS-Denied Vision-Based Autonomous UGV Navigation to Hospital Point B.
Brings up Gazebo Locality World, Differential UGV, Visual SLAM, Perception AI, Nav2, and Dashboard.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_forest_gazebo = get_package_share_directory('forest_ugv_gazebo')
    pkg_forest_description = get_package_share_directory('forest_ugv_description')
    pkg_sih_navigation = get_package_share_directory('sih_ugv_navigation')
    pkg_forest_vis = get_package_share_directory('forest_visualization')

    # Launch Arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2 Visualizer')
    nav2_arg = DeclareLaunchArgument('nav2', default_value='true', description='Launch Nav2 Navigation Stack')
    dashboard_arg = DeclareLaunchArgument('dashboard', default_value='true', description='Launch HERCULES Mission Control Dashboard')
    headless_arg = DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless')
    camera_gui_arg = DeclareLaunchArgument('camera_gui', default_value='false', description='Show standalone camera desktop GUI')

    rviz = LaunchConfiguration('rviz')
    nav2 = LaunchConfiguration('nav2')
    dashboard = LaunchConfiguration('dashboard')
    headless = LaunchConfiguration('headless')
    camera_gui = LaunchConfiguration('camera_gui')

    # 1. World & URDF Paths
    world_path = os.path.join(pkg_forest_gazebo, 'worlds', 'real_satellite_terrain_world.sdf')
    xacro_file = os.path.join(pkg_forest_description, 'urdf', 'forest_ugv.urdf.xacro')
    rviz_config = os.path.join(pkg_forest_vis, 'rviz', 'locality_hospital_navigation.rviz')
    nav2_launch_file = os.path.join(pkg_sih_navigation, 'launch', 'locality_nav2.launch.py')

    robot_desc = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 2. Gazebo Sim Launch
    resource_paths = ':'.join([
        os.path.join(pkg_forest_gazebo, 'worlds'),
        os.path.join(pkg_forest_description, 'models'),
        os.path.join(pkg_forest_description, 'urdf'),
        '/home/ubuntu/sih_ws/src/forest_ugv_description/models',
        '/home/ubuntu/sih_ws/install/forest_ugv_description/share/forest_ugv_description/models',
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ])

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_path],
        additional_env={'GZ_SIM_RESOURCE_PATH': resource_paths},
        output='screen'
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. Gazebo to ROS 2 Parameter Bridge
    bridge_params = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/gps/ground_truth@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
    ]

    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_params,
        parameters=[{'use_sim_time': True}]
    )

    # 6. EKF Sensor Fusion Node
    ekf_config = os.path.join(pkg_sih_navigation, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )

    # 7. Vision Perception & Multi-Class Semantic Segmentation Node
    perception_node = Node(
        package='forest_perception',
        executable='perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. GPS-Free Visual SLAM Odometry Node (publishes map->odom TF & trajectory)
    visual_odometry_node = Node(
        package='forest_visual_slam',
        executable='visual_odometry_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. Traversability Occupancy Costmap Node (publishes /map)
    traversability_mapping_node = Node(
        package='forest_mapping',
        executable='traversability_mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. Nav2 Navigation Stack Bringup
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true'}.items(),
        condition=IfCondition(nav2)
    )

    # 11. Dynamic Point A -> Point B Mission Coordinator
    point_ab_mission_node = Node(
        package='sih_ugv_navigation',
        executable='point_ab_mission_coordinator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 12. Dynamic Obstacle Injection & Movement Engine
    dynamic_obstacle_node = Node(
        package='sih_ugv_navigation',
        executable='dynamic_obstacle_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 13. Digital Twin Performance & Ground Truth Evaluator
    digital_twin_evaluator_node = Node(
        package='forest_evaluation',
        executable='digital_twin_evaluator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 14. HERCULES Mission Control Dashboard Web Server
    dashboard_server = Node(
        package='forest_dashboard',
        executable='dashboard_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(dashboard)
    )

    # 15. Google Gemini Multimodal VLA Brain Node
    gemini_brain_node = Node(
        package='forest_gemini_brain',
        executable='gemini_brain_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 16. RViz2 Professional Visualizer
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_locality',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    # 17. Live Desktop Camera Viewer GUI (Turned off by default)
    camera_gui_node = Node(
        package='forest_perception',
        executable='camera_viewer_gui',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(camera_gui)
    )

    return LaunchDescription([
        rviz_arg,
        nav2_arg,
        dashboard_arg,
        headless_arg,
        gz_sim,
        robot_state_publisher,
        parameter_bridge,
        ekf_node,
        perception_node,
        visual_odometry_node,
        traversability_mapping_node,
        nav2_stack,
        point_ab_mission_node,
        dynamic_obstacle_node,
        digital_twin_evaluator_node,
        camera_gui_node,
        dashboard_server,
        gemini_brain_node,
        rviz2_node,
    ])
