from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='body',
            name='Body',
        ),
        Node(
            package='controller',
            executable='camera',
            name='Camera',
        ),
    ])
