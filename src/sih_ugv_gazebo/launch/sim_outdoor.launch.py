#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sih_ugv_description = get_package_share_directory('sih_ugv_description')
    pkg_sih_ugv_gazebo = get_package_share_directory('sih_ugv_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=join(pkg_sih_ugv_gazebo, 'worlds', 'outdoor_terrain.sdf'),
        description='Gazebo world file'
    )

    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')

    xacro_path = join(pkg_sih_ugv_description, 'urdf', 'sih_ugv.urdf.xacro')
    rviz_config_file = join(pkg_sih_ugv_description, 'config', 'view.rviz')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_path, ' sim:=true'])
        }]
    )

    # Gazebo Sim Process
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Spawn UGV Robot Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sih_ugv',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.12',
            '-Y', '0.0'
        ]
    )

    # ROS-GZ Parameter Bridge for all sensors, controls, and transforms
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/camera/depth_image', '/camera/depth/image_raw'),
        ]
    )

    # RViz2 (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        rviz_arg,
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        rviz_node,
    ])
