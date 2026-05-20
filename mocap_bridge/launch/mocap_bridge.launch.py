
"""
mocap_bridge.launch.py
-----------------------
Lança o nó mocap_bridge com os parâmetros definidos em config/params.yaml.
"""
 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
 
    config = os.path.join(
        get_package_share_directory('mocap_bridge'),
        'config',
        'params.yaml'
    )
 
    mocap_bridge_node = Node(
        package='mocap_bridge',
        executable='mocap_bridge_node',
        name='mocap_bridge',
        output='screen',
        parameters=[config]
    )
 
    return LaunchDescription([
        mocap_bridge_node
    ])
