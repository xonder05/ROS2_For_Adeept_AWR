from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adeept_awr_input_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ultrasonic_node = adeept_awr_input_sensors.ultrasonic_node:main",
            "pi_camera_node = adeept_awr_input_sensors.pi_camera_node:main",
            "line_tracking_node = adeept_awr_input_sensors.line_tracking_node:main",
            "sound_recorder_node = adeept_awr_input_sensors.sound_recorder_node:main"
        ],
    },
)
