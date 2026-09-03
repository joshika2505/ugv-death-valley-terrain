import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_small_mountain = get_package_share_directory('small_mountain_world')
    pkg_amr4_description = get_package_share_directory('amr4_description')
    pkg_amr4_gazebo = get_package_share_directory('amr4_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Mountain plateau coordinates
    start_x = LaunchConfiguration('start_x', default='12.78')
    start_y = LaunchConfiguration('start_y', default='-6.80')
    start_z = LaunchConfiguration('start_z', default='464.5')
    start_yaw = LaunchConfiguration('start_yaw', default='0.0')

    world_sdf = os.path.join(pkg_small_mountain, 'worlds', 'small_mountain.sdf')
    bridge_config = os.path.join(pkg_amr4_gazebo, 'config', 'ros_gz_bridge.yaml')

    try:
        pkg_ugv_belt = get_package_share_directory('ugv_belt_drive')
    except Exception:
        pkg_ugv_belt = pkg_amr4_description

    share_dir = os.path.dirname(pkg_amr4_description)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            share_dir,
            pkg_ugv_belt,
            pkg_amr4_description,
            os.path.join(pkg_amr4_description, 'models'),
            pkg_small_mountain,
            os.path.join(pkg_small_mountain, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_sdf}.items()
    )

    robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr4_description, 'launch', 'rsp.launch.py')
        )
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'amr4',
            '-topic', 'robot_description',
            '-x', start_x,
            '-y', start_y,
            '-z', start_z,
            '-Y', start_yaw
        ]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    clock_filter_node = Node(
        package='amr4_autonomy',
        executable='clock_filter',
        name='clock_monotonic_filter',
        output='screen'
    )

    odom_tf_node = Node(
        package='amr4_autonomy',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='12.78', description='Initial X position'),
        DeclareLaunchArgument('start_y', default_value='-6.80', description='Initial Y position'),
        DeclareLaunchArgument('start_z', default_value='464.5', description='Initial Z elevation'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='Initial Yaw orientation'),
        gz_resource_path,
        gz_sim,
        robot_desc_launch,
        spawn_robot,
        ros_gz_bridge,
        clock_filter_node,
        odom_tf_node
    ])
