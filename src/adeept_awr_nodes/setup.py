from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'adeept_awr_nodes'

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
    maintainer_email='daniel.onderk@gmail.com',
    description='Contains nodes for controling hardware componennts of the Adeept AWR 4WD robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dc_motor_node = adeept_awr_nodes.dc_motor_node:main",
            "servo_node = adeept_awr_nodes.servo_node:main",
            "pi_camera_node = adeept_awr_nodes.pi_camera_node:main",
            "ultrasonic_node = adeept_awr_nodes.ultrasonic_node:main",
            "line_tracking_node = adeept_awr_nodes.line_tracking_node:main",
            "rgb_led_node = adeept_awr_nodes.rgb_led_node:main",
            "sound_transmitter_node = adeept_awr_nodes.sound_transmitter_node:main",
            "sound_receiver_node = adeept_awr_nodes.sound_receiver_node:main",
            "imu_node = adeept_awr_nodes.imu_node:main",
        ],
    },
)
