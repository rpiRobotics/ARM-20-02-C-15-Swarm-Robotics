# Husky Robot Setup Instructions
## 1. Industrial Computer Setup
When speccing out an industrial computer for use with the Husky make sure that it can be powered by 12V or 24V without exceeding the 5A current limit that the Husky's outside power ports supply. The industrial computer should also have a suitable number of USB ports to accomodate the serial to USB interface and any other sensors or additional inputs you wish to include and should be capable of running a ROS compatible linux operating system, preferably Ubuntu 18. Setup for this computer should be done outside the robot and is dependent on the user's preference, but a ROS compatible linux operating system should be installed and configured. ROS Melodic should also be installed and a catkin workspace created as described here: 

http://wiki.ros.org/melodic/Installation/Ubuntu

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Then the following commands should be Run:
`sudo apt-get install ros-melodic-husky-robot`

`rosrun husky_bringup install`

The installer will tell you to run one more command. Execute it.

Then to install the swarm control code to be used run the following commmands:

