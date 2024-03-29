#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.103" "192.168.1.104" "192.168.1.105")
USERNAMES=("tablet" "rockie" "oarbot_silver" "oarbot_blue" "administrator" "husky_blue" "husky_black")
PASSWORDS=("1234" "rockie" "1234" "1234" "clearpath" "1234" "1234")

SCRIPTS=(
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/noetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui;swarm_launch;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-noetic-spacenav-node;
    echo 1234 | sudo -S usermod -a -G dialout tablet
    "

    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    grep -qxF 'export ROS_IP=192.168.1.100' ~/.bashrc || echo 'export ROS_IP=192.168.1.100' >> ~/.bashrc;
    grep -qxF 'export ROSLAUNCH_SSH_UNKNOWN=1' ~/.bashrc || echo 'export ROSLAUNCH_SSH_UNKNOWN=1' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo rockie | sudo -S apt install -y spacenavd;
    echo rockie | sudo -S apt install -y ros-kinetic-spacenav-node;
    echo rockie | sudo -S usermod -a -G dialout rockie
    "

    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_silver
    "
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S usermod -a -G dialout oarbot_blue
    "
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo clearpath | sudo -S usermod -a -G dialout administrator
    "
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S usermod -a -G dialout husky_blue
    "
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control;arduino_talker';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S usermod -a -G dialout husky_black
    ")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    ssh-keygen -f "$HOME/.ssh/known_hosts" -R ${HOSTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o HostKeyAlgorithms='ssh-rsa' -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
