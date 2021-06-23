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

class OarbotControl_InvKin():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_inv_kin', anonymous=True)

        self.motor_cmd = MotorCmd()
        self.motor_command_name=rospy.get_param('~motor_command_topic_name')
        self.teleop_command_name=rospy.get_param('~republished_spacemouse')
 

        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)

      
        rospy.Subscriber(self.teleop_command_name, Twist, self.callback, queue_size=1)
        self.oarbot = OarbotKinematics()
        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        self.u4 = 0

    def callback(self, msg):
        self.inverse_kin(msg)

    def inverse_kin(self, msg):
        v_lin = msg.linear
        v_ang = msg.angular
        v_lin.x = -v_lin.x
        v_lin.y = -v_lin.y
        
        self.u1 = 1/self.oarbot.r * (v_lin.x + v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of front right motor
        self.u2 = 1/self.oarbot.r * (v_lin.x - v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of rear right motor
        self.u3 = 1/self.oarbot.r * (v_lin.x + v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of rear left motor
        self.u4 = 1/self.oarbot.r * (v_lin.x - v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        

        self.u1 = self.u1*60/(2*math.pi)
        self.u2 = self.u2*60/(2*math.pi)
        self.u3 = self.u3*60/(2*math.pi)
        self.u4 = self.u4*60/(2*math.pi)

        self.motor_cmd.v_fl = -self.u1
        self.motor_cmd.v_fr = self.u2
        self.motor_cmd.v_rl = self.u3
        self.motor_cmd.v_rr = -self.u4

        self.motor_cmd_pub.publish(self.motor_cmd)
        

if __name__ == "__main__":
    oarbotControl_InvKin = OarbotControl_InvKin()
    rospy.spin()