#!/bin/bash
HOSTS=("192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.103")
USERNAMES=("rockie" "oarbot_silver" "oarbot_blue" "administrator")
PASSWORDS=("rockie" "1234" "1234" "clearpath")

SCRIPTS=("cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/melodic/setup.bash;
    catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;"

    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;"
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;"
    
    "cd; 
    mkdir catkin_ws; 
    cd catkin_ws; 
    rm -rf {*,.*};
    git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git .;
    source /opt/ros/kinetic/setup.bash;
    catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control';
    grep -qxF 'source ~/catkin_ws/devel/setup.bash' ~/.bashrc || echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc;
    source ~/.bashrc;
    source ~/catkin_ws/devel/setup.bash;")

echo ${SCRIPTS}
for i in ${!HOSTS[*]} ; do
    echo "------------"
    # echo ${HOSTS[i]}
    echo ${USERNAMES[i]}
    # echo ${PASSWORDS[i]}
    echo ${SCRIPTS[i]}
    # sudo apt-get install sshpass
    sshpass -p ${PASSWORDS[i]} ssh -o StrictHostKeyChecking=no -o ConnectTimeout=2 -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
    # ssh -o StrictHostKeyChecking=no -l ${USERNAMES[i]} ${HOSTS[i]} "${SCRIPTS[i]}"
done

# echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc; 
