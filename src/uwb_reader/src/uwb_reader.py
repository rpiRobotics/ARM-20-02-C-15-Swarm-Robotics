#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import serial
import time

'''
uwb_reader.py
Alex Elias

Sends UWB LEC readings to ROS ropic as a string

Parameters:
    serial_port: e.g. '/dev/ttyS0'
    topic_name:  e.g. 'uwb_serial_front'
'''


class Uwb_reader:
    def __init__(self):
        rospy.init_node('uwb_reader', anonymous=True, disable_signals=True)
        self.serial_port = rospy.get_param('~serial_port')
        topic_name = rospy.get_param('~topic_name')

        self.ser = None

        self.pub = rospy.Publisher(topic_name, String, queue_size=1)
        
        rospy.on_shutdown(self.close_serial_if_active)

    def close_serial_if_active(self):
        if(not(self.ser == None)):
            self.ser.close()

    def start_lec_mode(self):
        rospy.loginfo("start_lec_mode start")
        rospy.loginfo("Reading first line:")
        ser_bytes = self.ser.readline()
        rospy.loginfo(ser_bytes)
        ser_bytes2 = self.ser.readline()
        rospy.loginfo(ser_bytes2)
        
        if "," in ser_bytes or "," in ser_bytes2: # already in terminal mode
            pass
        else: # need to start terminal mode
            # Two enter presses puts us into terminal mode
            self.ser.write('\r')
            self.ser.write('\r')
            
            # Wait until all the startup stuff is done
            for i in range(15):
                ser_bytes = self.ser.readline()
                rospy.loginfo(ser_bytes)
                if "dwm> " in ser_bytes:
                    break

            # Tell UWB tag to give us distance readings
            if not "DIST" in ser_bytes:
                self.ser.write("lec\r")
            ser_bytes = self.ser.readline() 
            rospy.loginfo(ser_bytes)

            # Throw out first reading (has extra "dwm> ")
            ser_bytes = self.ser.readline() 
            rospy.loginfo(ser_bytes)

            rospy.loginfo("start_lec_mode complete")

    def start_reading(self):
        while not rospy.is_shutdown():
            try:
                if(self.ser == None):
                    rospy.loginfo("Trying to reconnect to serial")
                    self.ser = serial.Serial(self.serial_port, 115200, timeout=0.5, xonxoff=True)
                    rospy.loginfo("Connected to serial")
                    time.sleep(1)
                    self.start_lec_mode()

                ser_bytes = self.ser.readline()
                if(ser_bytes):
                    self.pub.publish(ser_bytes)
                else:
                    rospy.logwarn("Serial timeout occured")

            except serial.serialutil.SerialException:
                if(not(self.ser == None)):
                    self.ser.close()
                    self.ser = None
                    rospy.logwarn("Disconnecting from serial")
                rospy.logwarn("Serial disconnected")
                time.sleep(0.25)


if __name__ == '__main__':
    uwb_reader = Uwb_reader()
    uwb_reader.start_reading()
