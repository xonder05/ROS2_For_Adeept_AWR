#!/bin/bash

while
    read -p "Install ROS2 Iron? [y/n]: " install_ros
    ! [[ "$install_ros" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

while
    read -p "Install python modules used by controllers (including pyqt)? [y/n]: " install_python
    ! [[ "$install_python" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

while
    read -p "Install Gazebo? [y/n]: " install_gazebo
    ! [[ "$install_gazebo" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

while
    read -p "Setup workspace? [y/n]: " setup_workspace
    ! [[ "$setup_workspace" =~ ^[yn]$ ]]
do echo "Unexpected value entered!"; done

if [[ "$install_ros" =~ "y" ]]; then
    echo "Installing ROS2 Iron and its dependencies"
    sudo apt update && sudo apt install software-properties-common curl -y
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt install ros-iron-desktop python3-colcon-common-extensions -y

    sudo -u $SUDO_USER bash -c '
        grep -qF "source /opt/ros/iron/setup.bash" ~/.bashrc || echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
        grep -qF "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
        grep -qF "export GZ_SIM_RESOURCE_PATH=\"\$HOME/ros2_ws/src/gazebo_simulator_nodes\"" ~/.bashrc || echo "export GZ_SIM_RESOURCE_PATH=\"\$HOME/ros2_ws/src/gazebo_simulator_nodes\"" >> ~/.bashrc
    '
fi

if [[ "$install_python" =~ "y" ]]; then
    echo "Instaling pip packages"
    sudo apt update && sudo apt install python3-pip -y
    sudo -u $SUDO_USER bash -c '
        pip install PyQt5 pygame pynput
    '
fi

if [[ "$install_gazebo" =~ "y" ]]; then
    echo "Instaling Gazebo simulator"
    sudo apt update && sudo apt-get install ros-iron-ros-gz -y
fi

if [[ "$setup_workspace" =~ "y" ]]; then
    echo "Downloading and building workspace"
    sudo apt update && sudo apt install git -y
    sudo -u $SUDO_USER bash -c '
        source /opt/ros/iron/setup.bash;
        mkdir ros2_ws && cd ros2_ws;
        git clone https://github.com/xonder05/ROS2_Adeept_AWR.git .;
        colcon build --symlink-install;
        grep -qF "source $(pwd)/install/setup.bash" ~/.bashrc || echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    '
fi