#!/bin/bash
HOSTS=("192.168.1.99" "192.168.1.100" "192.168.1.101" "192.168.1.102" "192.168.1.103")
USERNAMES=("tablet" "rockie" "oarbot_silver" "oarbot_blue" "administrator")
PASSWORDS=("1234" "rockie" "1234" "1234" "clearpath")
SCRIPTS=("cd ~/catkin_ws; git reset --hard; git pull;"
    "cd ~/catkin_ws; git reset --hard; git pull;" 
    "cd ~/catkin_ws; git reset --hard; git pull;" 
    "cd ~/catkin_ws; git reset --hard; git pull;"
    "cd ~/catkin_ws; git reset --hard; git pull;")
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
