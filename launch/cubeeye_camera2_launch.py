import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cubeeye_camera',
            executable='cubeeye_camera_node',
            name='cubeeye_camera_node2',
            output='screen'
        )
    ])
