from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch talker with parameters
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'v': 2.0, 'd': 0.5}]
        ),

        # Launch relay
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            output='screen',
        )
    ])
