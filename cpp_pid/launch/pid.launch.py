from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pid',
            executable='cpp_pid_main',
            name='cpp_pid_main',
            output='screen'
        )
    ])