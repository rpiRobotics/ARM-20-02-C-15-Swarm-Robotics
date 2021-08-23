#!/bin/bash
HOSTS=("192.168.1.103" "192.168.1.104" "192.168.1.105")
USERNAMES=("administrator" "husky_blue" "husky_black")
PASSWORDS=("clearpath" "1234" "1234")

SCRIPTS=("echo 1234 | sudo -S systemctl restart ros"
        "echo 1234 | sudo -S systemctl restart ros;"
        "echo 1234 | sudo -S systemctl restart ros;")
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
