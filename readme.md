# ROS2 system for controling Adeept AWR 4WD



## Install
Use instalation scripts `setup_stationary_device.sh` and `setup_robot.sh`. The scripts will download required dependencies and setup ros2_ws folder with contents of this repository.

## Getting Started
After instalation open new terminal or source `~/.bashrc`. Folder `ros2_ws/launch` contains ROS2 launch files. To start controllers use `ros2 launch controllers_launch.py` and then run either gazebo `gazebo_simulation_launch.py world_select:=[main/wandering/line_tracking]` on stationary device or `adeept_robot_launch.py` on adeept robot.