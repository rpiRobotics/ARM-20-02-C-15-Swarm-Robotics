# The Devices In The Lab:

| Description         | Username      | IP            | Password  | OS           | ROS     |
| ---                 | ---           | ---           | ---       | ---          | ---     |
| Pendant Tablet      | tablet        | 192.168.1.99  | 1234      | Ubuntu 20.04 | Noetic  |
| Main Computer       | rockie        | 192.168.1.100 | rockie    | Ubuntu 16.04 | Kinetic |
| Oarbot (Silver)     | oarbot_silver | 192.168.1.101 | 1234      | Ubuntu 16.04 | Kinetic |
| Oarbot (Blue)       | oarbot_blue   | 192.168.1.102 | 1234      | Ubuntu 16.04 | Kinetic |
| Clearpath Ridgeback | administrator | 192.168.1.103 | clearpath | Ubuntu 16.04 | Kinetic |

With the devices above configurations, just run `./all_devices_initial_setup.bash`. It will install all the software automatically.

# Ridgeback setup for remote host:
run `sudo nano /usr/sbin/ros-start`

Change the line `export ROS_MASTER_URI=http://127.0.0.1:11311` to
```
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.103
```

# Namespacing the Ridgeback
Following https://www.clearpathrobotics.com/assets/guides/kinetic/ridgeback/startup.html

In the `/etc/ros/kinetic/ros.d` directory of the Ridgeback, move the existing files and copy the three files in the [`ridgeback_startup`](https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics/tree/main/ridgeback_startup) folder. These new launch files put all the Ridgeback nodes and topics into the `/ridgeback` namespace.

You can run this automatically by running `./ridgeback_namespacing.bash`. This will move the files currently in `/etc/ros/kinetic/ros.d` to the folder `/home/administrator/backup_ros_d`.