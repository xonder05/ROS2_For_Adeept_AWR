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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dc_motor_node = adeept_awr_output_devices.dc_motor_node:main",
            "rgb_led_node = adeept_awr_output_devices.rgb_led_node:main",
            "servo_node = adeept_awr_output_devices.servo_node:main",
            "sound_player_node = adeept_awr_output_devices.sound_player_node:main",
            "ultrasonic_node = adeept_awr_input_sensors.ultrasonic_node:main",
            "pi_camera_node = adeept_awr_input_sensors.pi_camera_node:main",
            "line_tracking_node = adeept_awr_input_sensors.line_tracking_node:main",
            "sound_recorder_node = adeept_awr_input_sensors.sound_recorder_node:main",
            "web_controller_node = adeept_awr_web_controller.web_controller_node:main",
        ],
    },
)
