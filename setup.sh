#!/bin/bash

while
    echo "Install ROS2 Iron?"
    echo "  d - Desktop version of ROS2 (Recomended for stationary device)"
    echo "  b - Base version of ROS2 (Recomended for robot)"
    echo "  n - Don't install ROS2"
    read -p ": " install_ros
    ! [[ "$install_ros" =~ ^[dbn]$ ]]
do echo "Unexpected value entered!"; done

while
    echo "Install python modules?"
    echo "  c - Modules used by controllers (Recomended for stationary device)"
    echo "  r - Modules for controlling hardware components (Recomended for robot)"
    echo "  a - Both previous options (Recomended if you want to run controller nodes on robot)"
    echo "  n - Skip modules installation"
    read -p ": " install_python
    ! [[ "$install_python" =~ ^[cran]$ ]]
do echo "Unexpected value entered!"; done

while
    echo "Where is root of the workspace?"
    echo "  w - current working directory"
    echo "  s - this script is located in the root (its original location)"
    echo "  p - prompt for path"
    read -p ": " workspace_location
    ! [[ "$workspace_location" =~ ^[wsp]$ ]]
do echo "Unexpected value entered!"; done

if [[ "$workspace_location" == "w" ]]; then
    workspace_location=$(pwd) 
fi

if [[ "$workspace_location" == "s" ]]; then
    workspace_location=$(dirname "$(readlink -f "$0")")
fi

if [[ "$workspace_location" == "p" ]]; then
    
    while
        read -p "Enter workspace directory path: " workspace_location

        [ ! -d "$workspace_location" ] && echo "Directory does not exist." && continue
        [ ! -d "$workspace_location/.git" ] && echo "Git repository not found." && continue
        [ ! -d "$workspace_location/src" ] && echo "src directory not found." && continue
        [ ! -d "$workspace_location/launch" ] && echo "launch directory not found." && continue
        [ ! -f "$workspace_location/readme.md" ] && echo "README.md file not found." && continue
    do pass 
    done

    workspace_location=$(realpath "$workspace_location")
fi

while
    read -p "Install Gazebo Simulator? (requires instalation of ros2_control in the next step) [y/n]: " install_gazebo
    ! [[ "$install_gazebo" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

while
    read -p "Install ros2_control? [y/n]: " install_ros2_control
    ! [[ "$install_ros2_control" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

while
    read -p "Install slam_toolbox and navigation2? [y/n]: " install_nav_map
    ! [[ "$install_nav_map" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

#generic
sudo apt update -y

#ros2 install
if [[ "$install_ros" =~ ^[db]$ ]]; then
    echo "Installing ROS2 Iron and its dependencies"
    sudo apt update && sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    if [[ "$install_ros" == "d" ]]; then
        sudo apt update && sudo apt install ros-iron-desktop python3-colcon-common-extensions -y
    fi

    if [[ "$install_ros" == "b" ]]; then
        sudo apt update && sudo apt install ros-iron-ros-base python3-colcon-common-extensions -y
    fi

    grep -qF "source /opt/ros/iron/setup.bash" ~/.bashrc || echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
    grep -qF "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
fi

if [[ "$install_python" =~ ^[cra]$ ]]; then
    echo "Instaling pip packages"
    sudo apt install python3-pip -y

    if [[ "$install_python" =~ ^[ar]$ ]]; then
        sudo apt install python3-opencv raspi-config portaudio19-dev python3-smbus -y
        pip install RPi.GPIO adafruit-pca9685 rpi-ws281x sounddevice mpu6050-raspberrypi transforms3d
        sudo raspi-config nonint do_legacy 0
    fi

    if [[ "$install_python" =~ ^[ac]$ ]]; then
        sudo apt install portaudio19-dev -y
        pip install PyQt5 pygame pynput sounddevice
    fi
fi

if [[ "$install_gazebo" == "y" ]]; then
    echo "Instaling Gazebo simulator"
    sudo apt update && sudo apt-get install ros-iron-ros-gz -y
    grep -qF "export GZ_SIM_RESOURCE_PATH=\"${workspace_location}/src/gazebo_simulator_nodes\"" ~/.bashrc || echo "export GZ_SIM_RESOURCE_PATH=\"${workspace_location}/src/gazebo_simulator_nodes\"" >> ~/.bashrc

    if [[ "$install_ros2_control" =~ "y" ]]; then
        sudo apt install ros-iron-gz-ros2-control -y
    fi
fi

if [[ "$install_ros2_control" == "y" ]]; then
    echo "Instaling ros2_control"
    sudo apt-get install ros-iron-ros2-control ros-iron-ros2-controllers libgpiod-dev -y
fi

if [[ "$install_nav_map" == "y" ]]; then
    echo "Instaling Mapping and Navigation Tools"
    sudo apt-get install ros-iron-slam-toolbox ros-iron-navigation2 ros-iron-nav2-bringup -y
fi

source /opt/ros/iron/setup.bash
cd "$workspace_location"
#sudo chmod 777 /dev/ttyUSB0
#rosdep install --from-paths src
colcon build --symlink-install;
grep -qF "source ${workspace_location}/install/setup.bash" ~/.bashrc || echo "source ${workspace_location}/install/setup.bash" >> ~/.bashrc