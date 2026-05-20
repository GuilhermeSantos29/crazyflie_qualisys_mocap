from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('qualisys_ros2'),
        'config', 'QuadrotorCalib.yaml'
    )
    return LaunchDescription([
        Node(
            package='qualisys_ros2',
            executable='qualisys_calib_node',
            name='qualisys_calib',
            output='screen',
            parameters=[{
                'calib_marker_pos_file': config,
                'zero_pose_dir': os.path.join(
                    get_package_share_directory('qualisys_ros2'),'calib')
            }]
        )
    ])
