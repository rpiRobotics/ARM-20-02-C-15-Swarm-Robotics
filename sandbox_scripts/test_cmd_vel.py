#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import threading

class CmdVelTalker():
    def __init__(self):
        rospy.init_node('cmd_vel_talker', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/tablet/deadman_switch_spacenav_twist", Twist, queue_size=1)
        self.direction = 1

        rospy.Timer(rospy.Duration(0.01), self.talk)
        rospy.Timer(rospy.Duration(0.5), self.switch)

    def talk(self,event):    
        msg = Twist()
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = self.direction * 0.1        

        self.cmd_vel_pub.publish(msg)

    def switch(self,event):
        self.direction *= -1 


if __name__ == "__main__":
    CmdVelTalker = CmdVelTalker()
    rospy.spin()