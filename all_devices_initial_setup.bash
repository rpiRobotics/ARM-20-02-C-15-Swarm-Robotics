#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.103")
USERNAMES=("tablet" "rockie" "oarbot_silver" "oarbot_blue" "administrator")
PASSWORDS=("1234" "rockie" "1234" "1234" "clearpath")

SCRIPTS=(
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/noetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-noetic-spacenav-node;
    grep -qxF '192.168.1.99 tablet20'           /etc/hosts || echo '192.168.1.99 tablet20'           | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.100 rockie16'          /etc/hosts || echo '192.168.1.100 rockie16'          | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.101 oarbot_silver-NUC' /etc/hosts || echo '192.168.1.101 oarbot_silver-NUC' | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.102 oarbot_blue-NUC'   /etc/hosts || echo '192.168.1.102 oarbot_blue-NUC'   | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.103 CPR-R100-0064'     /etc/hosts || echo '192.168.1.103 CPR-R100-0064'     | sudo tee -a /etc/hosts;"

    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo rockie | sudo -S apt install -y spacenavd;
    echo rockie | sudo -S apt install -y ros-kinetic-spacenav-node;
    grep -qxF '192.168.1.99 tablet20'           /etc/hosts || echo '192.168.1.99 tablet20'           | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.100 rockie16'          /etc/hosts || echo '192.168.1.100 rockie16'          | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.101 oarbot_silver-NUC' /etc/hosts || echo '192.168.1.101 oarbot_silver-NUC' | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.102 oarbot_blue-NUC'   /etc/hosts || echo '192.168.1.102 oarbot_blue-NUC'   | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.103 CPR-R100-0064'     /etc/hosts || echo '192.168.1.103 CPR-R100-0064'     | sudo tee -a /etc/hosts;"

    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-kinetic-spacenav-node;
    grep -qxF '192.168.1.99 tablet20'           /etc/hosts || echo '192.168.1.99 tablet20'           | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.100 rockie16'          /etc/hosts || echo '192.168.1.100 rockie16'          | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.101 oarbot_silver-NUC' /etc/hosts || echo '192.168.1.101 oarbot_silver-NUC' | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.102 oarbot_blue-NUC'   /etc/hosts || echo '192.168.1.102 oarbot_blue-NUC'   | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.103 CPR-R100-0064'     /etc/hosts || echo '192.168.1.103 CPR-R100-0064'     | sudo tee -a /etc/hosts;"
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo 1234 | sudo -S apt install -y spacenavd;
    echo 1234 | sudo -S apt install -y ros-kinetic-spacenav-node;
    grep -qxF '192.168.1.99 tablet20'           /etc/hosts || echo '192.168.1.99 tablet20'           | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.100 rockie16'          /etc/hosts || echo '192.168.1.100 rockie16'          | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.101 oarbot_silver-NUC' /etc/hosts || echo '192.168.1.101 oarbot_silver-NUC' | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.102 oarbot_blue-NUC'   /etc/hosts || echo '192.168.1.102 oarbot_blue-NUC'   | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.103 CPR-R100-0064'     /etc/hosts || echo '192.168.1.103 CPR-R100-0064'     | sudo tee -a /etc/hosts;"
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;
    echo clearpath | sudo -S apt install -y spacenavd;
    echo clearpath | sudo -S apt install -y ros-kinetic-spacenav-node;
    grep -qxF '192.168.1.99 tablet20'           /etc/hosts || echo '192.168.1.99 tablet20'           | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.100 rockie16'          /etc/hosts || echo '192.168.1.100 rockie16'          | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.101 oarbot_silver-NUC' /etc/hosts || echo '192.168.1.101 oarbot_silver-NUC' | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.102 oarbot_blue-NUC'   /etc/hosts || echo '192.168.1.102 oarbot_blue-NUC'   | sudo tee -a /etc/hosts;
    grep -qxF '192.168.1.103 CPR-R100-0064'     /etc/hosts || echo '192.168.1.103 CPR-R100-0064'     | sudo tee -a /etc/hosts;")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -t -o StrictHostKeyChecking=no -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
