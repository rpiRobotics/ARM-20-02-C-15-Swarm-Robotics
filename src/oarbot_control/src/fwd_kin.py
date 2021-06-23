#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from swarm_msgs.msg import MotorCmd
import math

class OarbotKinematics():
    def __init__(self):
        # robot kinematic parameters in m
        self.r = 0.1016 # wheel radius
        self.l = 0.21463 # half length of chassis
        self.w = 0.22008 # half width of the chassis
        self.chain_drive_ratio = 10.0/15.0# 0.71429 # gear ratio of chain drive
        self.gear_ratio = 1.0/81.0*200/16

class OarbotControl():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_fwd_kin', anonymous=True)

        self.vel_feedback = Twist()
        self.vel_feedback_name=rospy.get_param('~velocity_feedback_topic_name')

        self.vel_pub = rospy.Publisher(self.vel_feedback_name, Twist, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)

        self.oarbot = OarbotKinematics()

    def forward_kin(self,event):    
        self.vel_feedback.linear.x = -self.oarbot.r/4 * (u1a + u2a + u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.linear.y = self.oarbot.r/4 * (-u1a + u2a - u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.angular.z = self.oarbot.r/(4*(self.oarbot.l + self.oarbot.w)) * (-u1a + u2a + u3a - u4a) * \
        self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.motor_lock.release()
        self.vel_pub.publish(self.vel_feedback)

if __name__ == "__main__":
    oarbot = OarbotControl()
    rospy.spin()
