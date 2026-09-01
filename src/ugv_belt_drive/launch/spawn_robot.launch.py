import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ugv_belt_drive')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ugv_belt_drive_robot.urdf.xacro')

    world_arg = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # If world_arg is not an absolute path or doesn't exist directly, resolve it
    if not os.path.isabs(world_arg) or not os.path.isfile(world_arg):
        candidate = os.path.join(pkg_share, 'worlds', world_arg)
        if os.path.isfile(candidate):
            world_file = candidate
        else:
            world_file = world_arg
    else:
        world_file = world_arg

    # Set Gazebo resource paths to resolve mesh and texture relative paths
    world_dir = os.path.dirname(os.path.abspath(world_file)) if os.path.isfile(world_file) else pkg_share
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    combined_res_path = f"{world_dir}:{pkg_share}:{gz_resource_path}".strip(':')

    gz_cmd = ['gz', 'sim', '-r']
    if headless:
        gz_cmd.append('-s')
    gz_cmd.append(world_file)

    gz_process = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': combined_res_path,
            'GZ_FILE_PATH': combined_res_path,
            'SDF_PATH': combined_res_path,
            'IGN_GAZEBO_RESOURCE_PATH': combined_res_path,
        }
    )

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 0.0
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', Command(['xacro ', xacro_file]),
            '-name', 'ugv_belt_drive',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity commands (both /ugv/cmd_vel and /cmd_vel)
            '/ugv/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry
            '/ugv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR sensor
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU sensor
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_tf_node = Node(
        package='ugv_belt_drive',
        executable='odom_to_tf_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        gz_process,
        robot_state_publisher,
        spawn_entity,
        bridge_node,
        odom_tf_node
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_belt_drive')
    default_world = os.path.join(pkg_share, 'worlds', 'terrain_world.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to Gazebo .world or .sdf file'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot initial X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot initial Y position'),
        DeclareLaunchArgument('z', default_value='2.0', description='Robot initial Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot initial Yaw orientation'),
        DeclareLaunchArgument('headless', default_value='false', description='Run simulation headless'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        OpaqueFunction(function=launch_setup)
    ])

