#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import serial
import time


class arduinoread:
	def __init__(self):
		rospy.init_node('arduino_talker', anonymous=True)
		self.arduino_topic=rospy.get_param('~arduino_topic')
		#self.arduino_topic="arduino_pub"
		self.comport=rospy.get_param('~com_port')
		#self.comfreq=rospy.get_param('baudrate')
		self.ser = serial.Serial(self.comport,115200,timeout=0.04)
		#self.ser = serial.Serial('/dev/ttyACM0',115200,timeout=0.04)
		self.arduino_pub = rospy.Publisher(self.arduino_topic, Bool, queue_size=10)
		time.sleep(1)
		rospy.Timer(rospy.Duration(0.04),self.read_serial)


	def read_serial(self,event):
		if not rospy.is_shutdown():
			output=self.ser.read()
			z=Bool()
			if(int(output)==1):
				z.data=1
				self.arduino_pub.publish(z)
			else:
				z.data=0
				self.arduino_pub.publish(z)
			self.ser.reset_input_buffer()
		else:
			rospy.signal_shutdown("system shutdown")








if __name__ == '__main__':
	arduinoreader=arduinoread()

	# Subscribe to space mouse velocity commands
	
	# Publish command velocity
	

	rospy.spin()