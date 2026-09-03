from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e', # Opens keyboard control in a separate terminal window
        remappings=[
            ('/cmd_vel', '/ugv/cmd_vel')
        ]
    )

    return LaunchDescription([
        teleop_node
    ])
