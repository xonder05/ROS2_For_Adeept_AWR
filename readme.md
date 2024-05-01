# ROS2 system for controling Adeept AWR 4WD


## Getting Started

### Operating System
This repository is using ROS2 in Iron version. Compatible operatings systems are Ubuntu 22.04 in desktop version for stationary device and in server rpi version for robot.

### Install
The installation itself is done using `setup.sh` script. It will download all required dependencies and setup ROS2 workspace.

### Notes before install
 - The instalation will setup stuff to work for currently logged user. 
 - The location of the repository during instalation will become ROS2 workspace.
 - The pedefined options for robot don't setup leds and lidar. Reason being leds reqire edit of sudoers file and lidar needs to be connected during installation.

### Usage
 - After the instaliation is complete open new terminal or source `~/.bashrc`. 
 - Also camera won't work untill you restart robot.

Launching nodes can be done either individually or use launch files provided in `ros2_ws/launch` directory. 

 - `ros2 launch robot_state_publisher`
    - firs step is to launch robot state publisher because it is needed for allmost every launch file and can just run somewhere in background
    - `use_sim:=[true/false]` - default value is false
 - `ros2 launch controllers_launch.py`
    - starts user interface along with other 
 -  `adeept_robot_launch.py`
    - robot
 - `gazebo_simulation_launch.py`
    - `world_select:=[main/wandering/line_tracking]` - default is wandering
