#!/bin/bash
HOSTS=("192.168.1.103")
USERNAMES=("administrator")
PASSWORDS=("clearpath")

SCRIPTS=(
    "cd; 
    mkdir backup_ros_d; 
    echo clearpath | sudo -S mv -v /etc/ros/kinetic/ros.d/* ~/backup_ros_d/;
    echo clearpath | sudo -S cp -a ~/catkin_ws/ridgeback_startup/. /etc/ros/kinetic/ros.d/;
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