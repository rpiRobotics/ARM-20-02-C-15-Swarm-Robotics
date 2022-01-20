# Husky Robot Setup Instructions
## 1. Industrial Computer Setup
When speccing out an industrial computer for use with the Husky make sure that it can be powered by 12V or 24V without exceeding the 5A current limit that the Husky's outside power ports supply. The industrial computer should also have a suitable number of USB ports to accomodate the serial to USB interface and any other sensors or additional inputs you wish to include and should be capable of running a ROS compatible linux operating system. Setup for this computer should be done outside the robot and is dependent on the user's preference, but a ROS compatible linux operating system should be installed and configured. Then the following commands should be Run:
`sudo apt-get install ros-melodic-husky-robot`

`rosrun husky_bringup install`

The installer will tell you to run one more command. Execute it.
