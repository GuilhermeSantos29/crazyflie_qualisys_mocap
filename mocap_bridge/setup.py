from setuptools import find_packages, setup
import os
from glob import glob
 
package_name = 'mocap_bridge'
 
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inclui os ficheiros de launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Inclui os ficheiros de config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gitonico',
    maintainer_email='guilherme.andre.santos@gmail.com',
    description='Bridge entre o MoCap Qualisys e o Crazyflie via ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_bridge_node = mocap_bridge.node:main',
            'teste_mocap_node = mocap_bridge.teste_mocap:main',
            'drone_controller_node = mocap_bridge.drone_controller_node:main',
            'cflib_controller_node = mocap_bridge.cflib_controller_node:main',
        ],
    },
)
