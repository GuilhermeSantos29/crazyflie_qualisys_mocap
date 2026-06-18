"""
qualisys.launch.py
------------------
Porta ROS 2 do launch file qualisys.launch do ROS 1.
Lança o nó qualisys_driver que liga ao QTM e publica as poses.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    qualisys_node = Node(
        package='qualisys_ros2',
        executable='qualisys_node',
        name='qualisys',
        output='screen',
        parameters=[{
            'server_address': '192.168.12.195',
            'server_base_port': 22222,
            'publish_tf': True,
            'frame_rate': 20,
        }]
    )

    return LaunchDescription([
        qualisys_node
    ])
