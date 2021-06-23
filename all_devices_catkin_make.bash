#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.103")
USERNAMES=("tablet" "rockie" "oarbot_silver" "oarbot_blue" "administrator")
PASSWORDS=("1234" "rockie" "1234" "1234" "clearpath")
SCRIPTS=(
    "source /opt/ros/noetic/setup.bash; cd; cd catkin_ws;  catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui'"
    "source /opt/ros/kinetic/setup.bash; cd; cd catkin_ws;  catkin_make -DCATKIN_WHITELIST_PACKAGES='swarm_msgs;swarm_gui;swarm_control;swarm_launch'"
    "source /opt/ros/kinetic/setup.bash; cd; cd catkin_ws;  catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch'"
    "source /opt/ros/kinetic/setup.bash; cd; cd catkin_ws;  catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch'"
    "source /opt/ros/kinetic/setup.bash; cd; cd catkin_ws;  catkin_make -DCATKIN_BLACKLIST_PACKAGES='swarm_gui;swarm_control;swarm_launch;oarbot_control'")
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
