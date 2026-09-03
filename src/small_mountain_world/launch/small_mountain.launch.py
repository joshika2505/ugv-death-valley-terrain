import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_small_mountain = get_package_share_directory('small_mountain_world')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_sdf = os.path.join(pkg_small_mountain, 'worlds', 'small_mountain.sdf')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
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

    return LaunchDescription([
        gz_resource_path,
        gz_sim
    ])
