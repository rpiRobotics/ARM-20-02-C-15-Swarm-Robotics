#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

class CmdVelTalker():
    def __init__(self):
        rospy.init_node('cmd_vel_talker', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher("/oarbot_silver/e_stop_cmd_vel", Twist, queue_size=1)
        self.direction = 1

        rospy.Timer(rospy.Duration(0.5), self.switch)

        self.duration = 0.03333 # loop duration
        print(time.time()) # float in second
        self.last_start_time = time.time()

    def talk(self):    
        start_time = time.time()
        print(start_time - self.last_start_time)
        self.last_start_time = start_time

        # if (start_time - self.last_start_time) > (self.duration*1.01):
            # rospy.logwarn("After sleep the loop takes more than defined duration time: " + str(self.duration) +" seconds." )

        msg = Twist()
        msg.linear.x = 0.
        msg.linear.y = 0.
        msg.linear.z = 0.
        msg.angular.x = 0.
        msg.angular.y = 0.
        msg.angular.z = self.direction * 0.5        

        self.cmd_vel_pub.publish(msg)

        finish_time = time.time()
        if (finish_time - start_time) < self.duration:
            time.sleep(self.duration*1.00 - (finish_time - start_time))
        else:
            rospy.logwarn("The loop takes more than defined duration time: " + str(self.duration) +" seconds." )

        

    def switch(self,event):
        self.direction *= -1 


if __name__ == "__main__":
    CmdVelTalker = CmdVelTalker()
    while not rospy.is_shutdown():
        CmdVelTalker.talk()