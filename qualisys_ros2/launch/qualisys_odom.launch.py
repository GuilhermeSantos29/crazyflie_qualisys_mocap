"""
qualisys_odom.launch.py
-----------------------
Porta ROS 2 do qualisys_odom.launch do ROS 1.
Lança o nó de odometria para um modelo específico.

Uso:
  ros2 launch qualisys_ros2 qualisys_odom.launch.py model:=cf231
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'model',
            description='Nome do modelo/rigid body'
        ),

        DeclareLaunchArgument(
            'qualisys_fps',
            default_value='100.0',
            description='Frequência do sistema Qualisys em Hz'
        ),

        Node(
            package='qualisys_ros2',
            executable='qualisys_odom_node',
            name=LaunchConfiguration('model'),
            output='screen',
            parameters=[{
                'qualisys_fps': LaunchConfiguration('qualisys_fps'),
            }],
            # Equivalente ao <remap> do ROS 1
            remappings=[
                ('qualisys_subject', ['qualisys/', LaunchConfiguration('model')])
            ]
        )
    ])
