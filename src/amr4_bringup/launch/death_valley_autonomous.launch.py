import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_amr4_bringup = get_package_share_directory('amr4_bringup')
    pkg_amr4_gazebo = get_package_share_directory('amr4_gazebo')
    pkg_amr4_nav = get_package_share_directory('amr4_navigation')
    pkg_amr4_autonomy = get_package_share_directory('amr4_autonomy')

    start_x = LaunchConfiguration('start_x', default='0.0')
    start_y = LaunchConfiguration('start_y', default='0.0')
    start_z = LaunchConfiguration('start_z', default='2.70')
    start_yaw = LaunchConfiguration('start_yaw', default='0.6')
    goal_x = LaunchConfiguration('goal_x', default='15.0')
    goal_y = LaunchConfiguration('goal_y', default='15.0')
    goal_yaw = LaunchConfiguration('goal_yaw', default='0.785')
    camera_gui = LaunchConfiguration('camera_gui', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    auto_nav = LaunchConfiguration('auto_nav', default='false')

    rviz_config = os.path.join(pkg_amr4_bringup, 'rviz', 'amr4_perception.rviz')

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr4_gazebo, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'start_x': start_x,
            'start_y': start_y,
            'start_z': start_z,
            'start_yaw': start_yaw
        }.items()
    )

    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr4_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen'
    )

    perception_node = Node(
        package='amr4_autonomy',
        executable='perception_display_node',
        name='perception_display_node',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'use_sim_time': True
        }],
        condition=IfCondition(camera_gui),
        output='screen'
    )

    mesh_path = os.path.join(get_package_share_directory('death_valley_world'), 'models', 'death_valley_terrain', 'meshes', 'death_valley_visual.obj')

    autonomy_node = Node(
        package='amr4_autonomy',
        executable='navigation_manager_node',
        name='navigation_manager_node',
        parameters=[{
            'auto_start': auto_nav,
            'web_port': 8080,
            'mesh_path': mesh_path,
            'use_sim_time': True
        }],
        output='screen'
    )

    try:
        pkg_ugv_nav = get_package_share_directory('autonomous_ugv_nav')
        renz_ugv_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ugv_nav, 'launch', 'ugv_navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'pointcloud_topic': '/scan'
            }.items()
        )
        renz_action = [TimerAction(period=3.5, actions=[renz_ugv_nav])]
    except Exception:
        renz_action = []

    return LaunchDescription([
        DeclareLaunchArgument('start_x', default_value='0.0', description='Point A X coordinate (Start)'),
        DeclareLaunchArgument('start_y', default_value='0.0', description='Point A Y coordinate (Start)'),
        DeclareLaunchArgument('start_z', default_value='2.80', description='Point A Z elevation'),
        DeclareLaunchArgument('start_yaw', default_value='0.6', description='Point A Yaw orientation'),
        DeclareLaunchArgument('goal_x', default_value='15.0', description='Point B Goal X coordinate (Stop)'),
        DeclareLaunchArgument('goal_y', default_value='15.0', description='Point B Goal Y coordinate (Stop)'),
        DeclareLaunchArgument('goal_yaw', default_value='0.785', description='Point B Goal Yaw angle'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('camera_gui', default_value='false', description='Open standalone camera POV window'),
        DeclareLaunchArgument('auto_nav', default_value='true', description='Auto-dispatch Point B goal'),

        gazebo_bringup,
        TimerAction(period=2.0, actions=[autonomy_node]),
        TimerAction(period=3.0, actions=[nav_bringup]),
        TimerAction(period=5.0, actions=[rviz_node, perception_node])
    ] + renz_action)
