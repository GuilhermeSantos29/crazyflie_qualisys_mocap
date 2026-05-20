"""
cflib_controller.launch.py
--------------------------
Lança o nó cflib_controller com os parâmetros definidos em config/params_cflib.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('mocap_bridge'),
        'config',
        'params_cflib.yaml'
    )

    cflib_controller_node = Node(
        package='mocap_bridge',
        executable='cflib_controller_node',
        name='cflib_controller',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        cflib_controller_node
    ])
