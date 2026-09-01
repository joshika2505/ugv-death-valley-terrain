#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = get_package_share_directory('forest_ugv_description')
    pkg_gazebo = get_package_share_directory('forest_ugv_gazebo')

    # Arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=join(pkg_gazebo, 'worlds', 'forest_world.sdf'),
        description='Gazebo world SDF path'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Initial X spawn position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Initial Y spawn position')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Initial Z spawn position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial Yaw spawn angle')

    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    xacro_path = join(pkg_desc, 'urdf', 'forest_ugv.urdf.xacro')
    rviz_config_file = join(pkg_desc, 'config', 'view.rviz')

    robot_desc_str = ParameterValue(
        Command(['xacro ', xacro_path, ' sim:=true']),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc_str
        }]
    )

    # Gazebo Sim Process
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'forest_ugv',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ]
    )

    # ROS-GZ Parameter Bridge for Camera, IMU, Control, Odometry, and Ground Truth GPS
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
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/ground_truth@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
        ]
    )

    # Optional RViz2
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
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        rviz_node,
    ])
