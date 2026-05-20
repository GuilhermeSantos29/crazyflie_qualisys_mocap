"""
rosbag_record.launch.py
-----------------------
Grava automaticamente os tópicos de posição:
  - /poses       → posição raw do MoCap (motion_capture_tracking)
  - /cf231/pose  → posição validada pelo mocap_bridge
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import datetime


def generate_launch_description():

    # Nome do ficheiro de gravação com timestamp automático
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name = f'voo_{timestamp}'

    return LaunchDescription([

        DeclareLaunchArgument(
            'bag_name',
            default_value=bag_name,
            description='Nome do ficheiro de gravação'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', LaunchConfiguration('bag_name'),
                '/poses',
                '/cf231/pose',
            ],
            output='screen'
        )
    ])
