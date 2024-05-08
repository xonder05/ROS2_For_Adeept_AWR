# ROS2 system for controlling Adeept AWR 4WD
This repository contains a ROS2 system for controlling the Adeept AWR 4WD robot, with additional hardware enhancements like an IMU sensor. It uses slam_toolbox and Nav2 libraries for mapping and navigation purposes.

Author: Daniel Onderka

This repository is part of bachelor's thesis.

### Link to git repository
https://github.com/xonder05/ROS2_For_Adeept_AWR

## Getting Started

### Operating System
This repository is using Iron distribution of ROS2. Recommended operating system for this distribution is Ubuntu 22.04. Stationary device is using desktop version of the system and robot uses Raspberry Pi, server version.

### Install
The installation itself is done using `setup.sh` script. It will download all required dependencies and setup ROS2 workspace.

1. Clone this repository
   - location of the repository during installation will become ROS2 workspace
2. Run the script and follow the instructions
   - script allows predefined install for robot or stationary device
   - manual picking of parts to be installed is also possible
   - predefined options for robot don't setup leds and lidar. Reasons being, leds setup edits sudoers file, and lidar setup requires lidar to be connected during installation
   - installation will setup stuff to work for currently logged user
3. After install
   - open new terminal or source `~/.bashrc`. 
   - camera won't work until you restart robot

### Usage
Launching nodes can be done either individually or use launch files provided in `workspace/launch` directory. 

- `ros2 launch robot_state_publisher`
   - first step is to launch robot state publisher, because it is needed for many different nodes
   - contains `use_sim:=[true/false]` parameter - default value is false
- `ros2 launch controllers_launch.py`
   - starts controllers and user interface
- `ros2 launch adeept_robot_launch.py`
   - for use on robot, launches all hardware nodes
   - starting of this file can be pretty slow (like a minute) so it is often better to just launch few currently needed nodes one by one
- `ros2 launch gazebo_simulation_launch.py`
   - launches simulator, bridges and compatibility nodes
   - `world_select:=[wandering/line_tracking/garden]` - default is wandering
- `ros2 launch controllers slam_mapping_launch.py`
   - starts slam_toolbox with custom configuration
   - `use_sim_time:=[true/false]` - default value is false
- `ros2 launch controllers navigation_launch.py`
   - launches nav2 with custom configuration
   - `use_sim_time:=[true/false]` - default value is false
   - for use in simulator set `params_file` to `simulator_custom_nav2_params.yaml` which is located in `controller/config` folder 
- `ros2 launch controllers rviz_launch.py`
   - starts rviz with custom configuration, for displaying maps and controlling navigation
   - `use_sim_time:=[true/false]` - default value is false
