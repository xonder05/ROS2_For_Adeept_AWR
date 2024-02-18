from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_simulator_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/world.sdf']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel.onderk@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simulator_motor = gazebo_simulator_nodes.simulator_motor:main",
            "simulator_camera_servo = gazebo_simulator_nodes.simulator_camera_servo:main",
            "simulator_camera = gazebo_simulator_nodes.simulator_camera:main",
            "simulator_ultrasonic = gazebo_simulator_nodes.simulator_ultrasonic:main",
            "simulator_line_tracking = gazebo_simulator_nodes.simulator_line_tracking:main",
        ],
    },
)
