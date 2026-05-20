"""
calibrate.launch.py
-------------------
Porta ROS 2 do calibrate.launch do ROS 1.
Lança o driver Qualisys e o nó de calibração em conjunto.

Uso:
  ros2 launch qualisys_ros2 calibrate.launch.py model:=cf231
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    calib_file = os.path.join(
        get_package_share_directory('qualisys_ros2'),
        'config',
        'QuadrotorCalib.yaml'
    )

    zero_pose_dir = os.path.join(
        get_package_share_directory('qualisys_ros2'),
        'calib'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'model',
            default_value='cf231',
            description='Nome do modelo/rigid body'
        ),

        # Equivalente ao nó vicon do ROS 1 — substituído pelo qualisys
        Node(
            package='qualisys_ros2',
            executable='qualisys_node',
            name='qualisys',
            output='screen',
            parameters=[{
                'server_address': '192.168.12.1',
                'server_base_port': 22222,
                'publish_tf': False,
            }]
        ),

        # Equivalente ao nó vicon_calibrate do ROS 1
        Node(
            package='qualisys_ros2',
            executable='qualisys_calib_node',
            name='qualisys_calib',
            output='screen',
            parameters=[{
                'calib_marker_pos_file': calib_file,
                'zero_pose_dir': zero_pose_dir,
            }],
            remappings=[
                ('qualisys_calib',   ['qualisys/', 'QuadrotorCalib']),
                ('qualisys_subject', ['qualisys/', LaunchConfiguration('model')]),
            ]
        )
    ])
