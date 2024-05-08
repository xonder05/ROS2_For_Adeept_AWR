from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, package_name, 'user_interface'), glob(os.path.join(package_name, 'user_interface', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='daniel.onderk@gmail.com',
    description='Contains user inteface and manual (gamepad and keyboard) as well as automatic (wandering, line following) nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wandering_node = controllers.wandering_node:main",
            "line_following_node = controllers.line_following_node:main",
            "benchmark_node = controllers.benchmark_node:main",
            "gamepad_node = controllers.gamepad_node:main",
            "keyboard_node = controllers.keyboard_node:main",
            "user_interface = controllers.user_interface.user_interface:main",
            "motor_controller_node = controllers.motor_controller_node:main",
        ],
    },
)
