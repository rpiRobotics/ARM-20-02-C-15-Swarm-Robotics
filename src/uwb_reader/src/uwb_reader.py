#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import serial

'''
uwb_reader.py
Alex Elias

Sends UWB LEC readings to ROS ropic as a string

Parameters:
	serial_port: e.g. '/dev/ttyS0'
	topic_name:  e.g. 'uwb_serial_front'
'''

def turn_off_lec(ser):
	
	# # Turn off lec
	# ser.write('\r')
	# # This turns the terminal mode off
	# ser.write('quit\r')
	# ser_bytes = ser.readline() 
	# print(ser_bytes)
	# ser_bytes = ser.readline() 
	# print(ser_bytes)
	# ser_bytes = ser.readline() 
	# print(ser_bytes)
	# ser_bytes = ser.readline() 
	# print(ser_bytes)

	ser.close()

if __name__ == '__main__':
	try:
		print("Starting uwb_reader ROS node")	
		rospy.init_node('uwb_reader', anonymous=True, disable_signals=True)
		serial_port = rospy.get_param('~serial_port')
		topic_name = rospy.get_param('~topic_name')

		print("Starting serial port")
		ser = serial.Serial(serial_port, 115200, timeout=1, xonxoff=True)

		pub = rospy.Publisher(topic_name, String, queue_size=1)
		
		#rospy.on_shutdown(turn_off_lec)
		rospy.on_shutdown(ser.close)

		#ser.flush()
		
		print("Reading first line:")
		ser_bytes = ser.readline()
		ser_bytes2 = ser.readline()
		print("First read after flush:")
		print(ser_bytes)
		if "," in ser_bytes or "," in ser_bytes2: # already in terminal mode
			#pass
			print("passing")
		else: # need to start terminal mode
			# Two enter presses puts us into terminal mode
			ser.write('\r')
			ser.write('\r')
			
			# Wait until all the startup stuff is done
			for i in range(15):
				ser_bytes = ser.readline()
				print(ser_bytes)
				if "dwm> " in ser_bytes:
					break

			# Tell UWB tag to give us distance readings
			if not "DIST" in ser_bytes:
				ser.write("lec\r")
			ser_bytes = ser.readline() 
			print(ser_bytes)

			# Throw out first reading (has extra "dwm> ")
			ser_bytes = ser.readline() 
			print(ser_bytes)

		while True:
			ser_bytes = ser.readline()
			if(ser_bytes):
				#rospy.loginfo(ser_bytes)
				pub.publish(ser_bytes)

	except KeyboardInterrupt:
		print('Interrupt!')
		#turn_off_lec(ser)
