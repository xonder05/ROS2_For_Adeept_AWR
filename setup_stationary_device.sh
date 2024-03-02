
#install ros2 iron
sudo apt update && sudo apt install software-properties-common curl git -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-iron-desktop python3-colcon-common-extensions -y

sudo -u $SUDO_USER bash -c '
    echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc;
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc;
'
#download and build workspace
sudo -u $SUDO_USER bash -c '
    source /opt/ros/iron/setup.bash;
    mkdir ros2_ws && cd ros2_ws;
    git clone https://github.com/xonder05/ROS2_Adeept_AWR.git .;
    colcon build --symlink-install;
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc;
'
#install gazebo
sudo apt-get install ros-iron-ros-gz -y

#install qt
sudo apt install python3-pip -y
sudo -u $SUDO_USER bash -c '
    pip install PyQt5 pygame pynput
'