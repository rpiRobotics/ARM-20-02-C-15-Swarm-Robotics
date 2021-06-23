The Devices In The Lab:
Pendant Tablet		=> Username: tablet, 		IP: 192.168.1.99,	Password: 1234,		Ubuntu 20.04 Desktop, ROS Noetic
Main Computer 		=> Username: rockie, 		IP: 192.168.1.100,	Password: rockie, 	Ubuntu 16.04 Desktop, ROS Kinetic
Oarbot (Silver)		=> Username: oarbot_silver, IP: 192.168.1.101,	Password: 1234, 	Ubuntu 16.04 Desktop, ROS Kinetic
Oarbot (Blue)		=> Username: oarbot_blue, 	IP: 192.168.1.102,	Password: 1234, 	Ubuntu 16.04 Desktop, ROS Kinetic
Clearpath Ridgeback	=> Username: administrator, IP: 192.168.1.103,	Password: clearpath, Ubuntu 16.04 Server, ROS Kinetic

With the devices above configurations, just run all_devices_initial_setup.bash. It will install all the software automatically.

Ridgeback setup for remote host:
run
	sudo nano /usr/sbin/ros-start

Change the line
	export ROS_MASTER_URI=http://127.0.0.1:11311
to
	export ROS_MASTER_URI=http://192.168.1.100:11311/
	export ROS_IP=192.168.1.103