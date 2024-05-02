# ROS2 system for controling Adeept AWR 4WD

## Getting Started

### Operating System
This repository is using Iron distribution of ROS2. Recomended operating system for this distribuiton is Ubuntu 22.04. Stationary device is using desktop version of the system and robot uses server rpi version.

### Install
The installation itself is done using `setup.sh` script. It will download all required dependencies and setup ROS2 workspace.

1. Clone this repository
   - location of the repository during instalation will become ROS2 workspace
2. Run the script and follow the instructions
   - script alows predefined install for robot or stationary device
   - manual choosing of parts to be installed is also possible
   - pedefined options for robot don't setup leds and lidar. Reasons being, leds setup edits sudoers file, and lidar setup requires lidar to be connected
   - instalation will setup stuff to work for currently logged user
3. After install
   - open new terminal or source `~/.bashrc`. 
   - camera won't work untill you restart robot

### Usage
Launching nodes can be done either individually or use launch files provided in `workspace/launch` directory. 

- `ros2 launch robot_state_publisher`
   - firs step is to launch robot state publisher, because it is needed for many different nodes
   - contains `use_sim:=[true/false]` param - default value is false
- `ros2 launch controllers_launch.py`
   - starts controllers and user interface
-  `ros2 launch adeept_robot_launch.py`
   - for use on robot, launches all hardware nodes
   - starting of this file can be pretty slow (like a minute) so it is often better to just launch few curently needed nodes one by one
- `ros2 launch gazebo_simulation_launch.py`
   - launches simulator with all bridges
   - `world_select:=[wandering/line_tracking/garden]` - default is wandering
- `ros2 launch controllers slam_mapping_launch.py`
   - starts slam_toolbox with custom config
   - `use_sim_time:=true`
- `ros2 launch controllers navigation_launch.py`
   - launches nav2 with custom config
   - `use_sim_time:=true`
- `ros2 launch controllers rviz_launch.py`
   - starts rviz with custom config, for displaying maps and controling navigation
   - `use_sim_time:=true`
