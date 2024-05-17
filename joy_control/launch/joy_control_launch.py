from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='joy_control',
            executable='joy_control',
            name='joy_control'
        ),
        Node(
            package='dbw',
            executable='dbw',
            name='dbw',
        )
    ])
