"""
drone_controller.launch.py
--------------------------
Lança o nó drone_controller com os parâmetros definidos em config/params_controller.yaml.
"""
 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
 
    config = os.path.join(
        get_package_share_directory('mocap_bridge'),
        'config',
        'params_controller.yaml'
    )
 
    drone_controller_node = Node(
        package='mocap_bridge',
        executable='drone_controller_node',
        name='drone_controller',
        output='screen',
        parameters=[config]
    )
 
    return LaunchDescription([
        drone_controller_node
    ])
 
