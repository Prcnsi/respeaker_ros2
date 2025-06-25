# audio_with_doa.launch.py

import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # AudioWithDOA 퍼블리셔 노드를 실행
        Node(
            package='respeaker_ros2',
            executable='audio_with_doa_node',
            name='audio_with_doa_publisher',
            output='screen'
        ),
    ])
