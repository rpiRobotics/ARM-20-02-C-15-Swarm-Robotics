# Husky Robot Setup Instructions
## 1. Industrial Computer Setup and Initial Software Install
When speccing out an industrial computer for use with the Husky make sure that it can be powered by 12V or 24V without exceeding the 5A current limit that the Husky's outside power ports supply. The industrial computer should also have a suitable number of USB ports to accomodate the serial to USB interface and any other sensors or additional inputs you wish to include and should be capable of running a ROS compatible linux operating system, preferably Ubuntu 18. Setup for this computer should be done outside the robot and is dependent on the user's preference, but a ROS compatible linux operating system should be installed and configured. ROS Melodic should also be installed and a catkin workspace created as described here: 

http://wiki.ros.org/melodic/Installation/Ubuntu

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Then the following commands should be Run:
`sudo apt-get install ros-melodic-husky-robot`

`rosrun husky_bringup install`

The installer will tell you to run one more command. Execute it.

Then to install the swarm control code to be used run the following commmands:

'cd <to catkin workspace directory>'
'cd src'
'git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git'

Be sure to then run 'catkin_make' in the catkin workspace directory to build the code.

  
## 2. Wireless Setup
In addition, it is recommended that an external wifi module is used, which should ideally be mounted above the robot to ensure a quality connection for controlling the robot and communicating with the ROS master. This should be plugged into the industrial computer and setup prior to installing the system in the robot. The details of this setup process are dependent on the configuration of the wireless network setup, but a static IP address should be given to the industrial PC to allow communication to the ROS master. 

The desired ROS master IP address should then be set on the industrial computer to be the static IP address of whatever central control computer is coordinating the robot system, in the following lines that IP is assumed to be 192.168.1.100, but that should be set up ahead of time. To change the ROS master IP address do the following: 
  
  
run `sudo nano /usr/sbin/ros-start`

Change the line `export ROS_MASTER_URI=http://127.0.0.1:11311` to
```
export ROS_MASTER_URI=http://192.168.1.100:11311/
export ROS_IP=192.168.1.103
```

To make sure that ros.service by Clearpath starts after the network is really online, 
edit `ros.service` file with command   
`sudo nano /lib/systemd/system/ros.service`   
and add the following lines   
```
After=network-online.target
Wants=network-online.target
```
in place of the line   
```
After=network.target
```
[For further information about this above see this link.](https://www.freedesktop.org/wiki/Software/systemd/NetworkTarget/)


  
