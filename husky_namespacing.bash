#!/bin/bash
HOSTS=("192.168.1.104" "192.168.1.105")
USERNAMES=("husky_blue" "husky_black")
PASSWORDS=("1234" "1234")

SCRIPTS=(
    "cd; 
    mkdir backup_ros_d; 
    echo 1234 | sudo -S mv -v /etc/ros/melodic/ros.d/* ~/backup_ros_d/;
    echo 1234 | sudo -S cp -a ~/catkin_ws/husky_blue_startup/. /etc/ros/melodic/ros.d/;
    "
    "cd; 
    mkdir backup_ros_d; 
    echo 1234 | sudo -S mv -v /etc/ros/melodic/ros.d/* ~/backup_ros_d/;
    echo 1234 | sudo -S cp -a ~/catkin_ws/husky_black_startup/. /etc/ros/melodic/ros.d/;
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