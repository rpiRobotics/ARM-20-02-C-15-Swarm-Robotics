# Husky Robot Setup Instructions
## 1. Industrial Computer Setup and Initial Software Install
When speccing out an industrial computer for use with the Husky make sure that it can be powered by 12V or 24V without exceeding the 5A current limit that the Husky's outside power ports supply. The industrial computer used in this project was purchased from Onlogic, specifically it was a Low Profile Fanless Industrial Computer With Apollo Lake, it's motherboard was an Intel Apollo Lake N3350 Industrial Motherboard with Single Gb LAN, Memory (RAM) 1 x 8 GB SO-DIMM DDR3L 1600, Primary Storage 1 x 256 GB mSATA SSD, with mounting brackets, with AC adapter. A link to purchase this industrial PC is here:

https://www.onlogic.com/ml350g-10/

The industrial computer should also have a suitable number of USB ports to accomodate the serial to USB interface and any other sensors or additional inputs you wish to include and should be capable of running a ROS compatible linux operating system, preferably Ubuntu 18. Setup for this computer should be done outside the robot and is dependent on the user's preference, but ideally the username chosen for the computer should match the chosen name of the robot, but a ROS compatible linux operating system should be installed and configured. ROS Melodic should also be installed and a catkin workspace created as described here: 

http://wiki.ros.org/melodic/Installation/Ubuntu

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Then the following commands should be Run:
`sudo apt-get install ros-melodic-husky-robot`

`rosrun husky_bringup install`

The installer will tell you to run one more command. Execute it.

Then to install the swarm control code to be used run the following commmands:

`cd to catkin workspace directory`


`cd src`


`git clone https://github.com/rpiRobotics/ARM-20-02-C-15-Swarm-Robotics.git`


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

## 3. Namespacing Setup
To allow for control of multiple Husky robots using namespacing then the files found in the husky_blue_startup or husky_black_startup should be moved into the '/etc/ros/melodic/ros.d' directory, this can be done automatically by running the husky_namespacing.bash executable file on the control computer once the correct IP addresses for both the control computer and the industrial computer have been setup. The setup and code in this repository was made assuming two Husky robots in the swarm, one named Husky_Blue, and the other named Husky_Black, to expand the number of Huskies in the swarm, all parameter files located in swarm_launch/config/ that reference Husky_Blue or Husky_Black would need to be duplicated and their parameters changed to reference a new Husky.   This process can be easily extended to more than two Huskies by editing that same bash file and expanding the three lists for HOSTS, which should be the static IP addresses of each Husky industrial computer, Usernames, which should be the usernames specified during the initial Ubuntu setup and user profile creation process (this is used to ssh into the husky and set these values correctly), and finally Passwords, which should be the password used to log into the specified user profile on the corresponding Husky industrial computer. This editing process will need to be repeated for any of the "all_devices....bash" scripts that are to be used, this is so that each device can be correctly SSH'd into and commands can be run easily In addition, these new Huskies would have to be added into the primary system launch file in `src/swarm_launch/launch/swarm_all.launch` and additional copies of husky_XXXX.launch would need to be added, as well as the parameters in the following files would need to be changed: gui_params.yaml, the swarm_control.yaml, state_machine_rf_welding.yaml, to show up correctly in the user interface and interact correctly with all the control systems.

## 4. Getting Power From Husky
To get power from the Husky to power the industrial computer, first follow the instructions in the Husky-UGV-User-Manual.pdf found in readme_include/ in this directory to remove the metal cover on top of the robot to gain access to the enclosure below which should look as shown below:


  ![Robots](/readme_include/Husky_view.PNG)


Inside the enclosure there are three sets of connecting ports with 3 power and ground pins each, one that supplies 5V power, one that supplies 12V and one that supplies 24V, they all can source up to 5 A at maximum draw, which should be kept in mind while wiring into these ports. These husky ports are detachable, and for convenience should be removed from the Husky before wiring into them. A section is devoted to this in the husky manual which discusses how to use a screwdriver to add wires into the ports as shown below:


![Husky Power Ports](/husky_images/husky_power_wiring.PNG)

Whatever power plug is used to power the industrial PC that was purchased might need to be cut up and wired into the appropriate power port.
## 5. Mounting industrial computer inside Husky
The inside of the husky enclosure there has no readily available mounting holes, so to successfully mount into the enclosure a plate must be 3D printed or lasercut to fit snugly inside the space with mounting holes that match the industrial computer chosen. An example of this mounting plate can be seen here and a 3D model of this part is available in Husky_CAD_files as "computer mounting plate.SLDPRT".


 ![Husky_computer_mounting_plate](/husky_images/computer%20mounting%20plate.JPG)

Depending on the form factor of the industrial PC chosen, the mounting holes in this plate may need to be modified. Before replacing the metal cover on the robot, make sure to plug in any USB extension modules as well as the USB-serial cable that connects the industrial PC into the Husky robot.
  
## 6. Attaching wifi module
After having inserted the industrial PC into the Husky, a USB wifi module should be mounted somewhere above the enclosure to limit interference from the metal cover of the robot. The wifi module can be mounted in any number of places, such as on the 3x3 tower strut, or on a 90 degree angle piece as shown below: 

![Husky_UWB_bracket](/husky_images/UWB%20brkt.JPG)

You should verify the integrity of the wifi connection by attempting to SSH into the robot using the static IP address and user name specified during the Industrial PC setup process.
 
## 7. Attaching UWB sensors
Attaching the UWB sensors involves using the same 90 degree angle part shown above, CAD for this part also exists in Husky_CAD_files as "UWB brkt.SLDPRT", these parts are meant to be mounted on 2 of the opposing corners of the Husky robot, since a minimum of two UWB sensors are needed to fully localize the robot in the workspace, as can be seen below: 

![Husky UWB View](/husky_images/2021-08-04-11-36-37-490.jpg)

The actual setup of the UWB modules is beyond the scope of these instructions, but documentation and usage guides for the UWB modules used can be found here:

https://www.decawave.com/product/mdek1001-deployment-kit/

In the context of the SWARM software used, it is important to remember to change the position of the tags relative to the center of the robot in the software. This is located in src/swarm_launch/config/husky_XXXX_sensor_fusion.yaml, the values tag_loc_front_x and tag_loc_front_y correspond to the value in meters of the offset between the location of the UWB tag and the center of the robot about which the robot moves. The software also assumes the tags to be at the same height, so they should be secured at the same height as much as possible. The tag location values can be found by measuring to the center of the robot and can be refined by testing and further calibration using the closed loop motion control. The ID of the tags should also be specified in that same file. In addition, a last bit of anchor calibration is performed according to a Decawave white paper included in this repository under readme_include/APS014_Antennna-Delay-Calibration_V1.2.pdf, and the offsets of each tag should be put into antenna_calibration.yaml.
  
## 8. USB port identifiers
To guarantee the correct UWB sensor is addressed, go into the file src/swarm_launch/config/husky_XXX_uwb_XXXX.yaml, and set the value of serial_port in the file to be equal to the value of the path to the usb port of the UWB sensor. This can be found by running the following commands:

`cd /dev/serial/by-path/`

`ls`

Then unplug the UWB sensor and call `ls` again, take note of which entry disappears, enter this entry into the yaml file as serial_port.
  
## 9. Taping Wheels
To improve traction of Husky on inside surfaces, the surface of the wheels can be covered in plain packing tape which should be wrapped a few layers deep to allow the robot to move on the tape rather than the treads, this especially helps on carpeted surfaces where the aggressive treads can limit ability of the robot to move as precisely. If operating outdoors this should be ignored. 
  
## 10. Attaching Fabric Engagement Tower
CAD files for all the parts in the Fabric Engagement Tower are available in Husky_CAD_Files, and an assembly displaying the entire tower can be seen in "Husky fabric engagement.SLDASM". Any CAD parts with a product code in front of them can be purchased with that code from McMaster-Carr.

https://www.mcmaster.com/

The first step to assembling the Fabric Engagement tower is to bolt the part called "lower plate.SLDPRT" to the surface of the Husky top metal cover on the side farthest from the battery hookup cover. Next the 3x3 piece of aluminum strut is attached to the lower plate, this piece of strut can have a variable height, but its CAD file is "3x3 x 23.SLDPRT". Next the part known as "brgplate.SLDPRT" is mounted to the side of the 3x3 aluminum strut facing toward the center of the robot. Next, two mounted ball bearing parts are attached to the brgplate, both should be "5913K61_Low-Profile Mounted Sealed Steel Ball Bearing.SLDPRT". Then a half inch diameter aluminum shaft, "shaft.SLDPRT" should be inserted into both ball bearing mounts and fixed using the set screws on both mounts. Then, at the desired height, a washdown set screw shaft collar, "60475K72_Washdown Set Screw Shaft Collar.SLDPRT" should be attached to allow the top plate, "top plate.SLDPRT", to rest on it. Then another shaft collar should be screwed on top of the top plate to fix it at that height. On the top plate, the angle part, "angle.SLDPRT" should first be attached, the angle part has channels so that it can be moved along the plate and fixed in desired positions. Next, both of the threaded stud bumpers, "9541K37_Threaded-Stud Bumper.SLDPRT" should be screwed onto the front of their respective clamps, "6124A21_Toggle Clamp with Locking Handle.SLDPRT". both clamps should then be attached to the top plate. The end result should look as shown below: 

![Husky Fabric Engagement tower](/husky_images/Husky%20fabric%20engagement_2.JPG)


## 11. Starting SWARM System
After performing all the previous steps described, make sure that you have run all_devices_initial_setup.bash which should pull all necessary github repositories into the respective Huskies, if any updates need to be pulled, run all_devices_git_pull.bash. Then run all_devices_catkin_make.bash to build all the directories in the Huskies. Finally starting from catkin_ws, run the following commands:

`cd src/ARM-20-02-C-15-Swarm-Robotics/src/swarm_launch/launch`
`roslaunch swarm_all.launch`

The Huskies should start, you should be able to run `rostopic list` and see all the corresponding topics related to each Husky robots with their correct namespacing, and in addition the communication light on the front of the Husky should have turned green to indicate that it is successfully communicating with the industrial computer. If either of these are not the case you should doublecheck that you followed the steps correctly, that you can ping the husky robot's ip address and SSH into it correctly, check all associated parameter files and make sure namespacing is consistent and that rosnodes are being started correctly. If you are having a particular issue with the code in this repository feel free to leave a github issue addressing the complaint with adequate detail as to the specifics of your issue.
