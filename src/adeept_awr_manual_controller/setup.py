from setuptools import find_packages, setup

package_name = 'adeept_awr_manual_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "keyboard_reader = adeept_awr_manual_controller.keyboard_reader:main",
            "led_color_changer = adeept_awr_manual_controller.led_color_changer:main",
            "servo_angle_changer = adeept_awr_manual_controller.servo_angle_changer:main"
        ],
    },
)
