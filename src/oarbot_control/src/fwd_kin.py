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

class OarbotControl_FwdKin():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_fwd_kin', anonymous=True)

        self.vel_feedback = Twist()

        self.vel_feedback_name=rospy.get_param('~velocity_feedback_topic_name')
        self.vel_pub = rospy.Publisher(self.vel_feedback_name, Twist, queue_size=1)

        self.motor_feedback_topic_name=rospy.get_param('~motor_feedback_topic_name')
        rospy.Subscriber(self.motor_feedback_topic_name, MotorCmd, self.motor_feedback_callback, queue_size=1)

        self.oarbot = OarbotKinematics()

    def motor_feedback_callback(self, msg):
        self.forward_kin(msg)


    def forward_kin(self,msg):    
        u1a = -msg.v_fl/60*2*math.pi
        u2a =  msg.v_fr/60*2*math.pi
        u3a =  msg.v_rl/60*2*math.pi
        u4a = -msg.v_rr/60*2*math.pi


        self.vel_feedback.linear.x = -self.oarbot.r/4 * (u1a + u2a + u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.linear.y = self.oarbot.r/4 * (-u1a + u2a - u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.angular.z = self.oarbot.r/(4*(self.oarbot.l + self.oarbot.w)) * (-u1a + u2a + u3a - u4a) * \
        self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.motor_lock.release()
        self.vel_pub.publish(self.vel_feedback)

if __name__ == "__main__":
    oarbotControl_FwdKin = OarbotControl_FwdKin()
    rospy.spin()
