#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from swarm_msgs.msg import Twist2D, MotorStatus, MotorCmd
from roboteq_handler import RoboteqHandler
import roboteq_commands as cmds
import math
import threading
import time

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
        self.motor_lock = threading.Lock()
        rospy.init_node('oarbot_ctrl', anonymous=True)

        self.vel_feedback = Twist()
        self.motor_cmd = MotorCmd()
        self.vel_feedback_name=rospy.get_param('~velocity_feedback_topic_name')
        self.motor_command_name=rospy.get_param('~motor_command_topic_name')
        self.teleop_command_name=rospy.get_param('~republished_spacemouse')
        serial_front = rospy.get_param('~serial_front')
        serial_back = rospy.get_param('~serial_back')

        self.vel_pub = rospy.Publisher(self.vel_feedback_name, Twist, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)

        # connection to Roboteq robot controller
        self.controller_f = RoboteqHandler(debug_mode=False, exit_on_interrupt=False)
        self.controller_b = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
        self.connected_f = self.controller_f.connect(serial_front)
        self.connected_b = self.controller_b.connect(serial_back)
        rospy.Subscriber(self.teleop_command_name, Twist, self.callback, queue_size=1)
        self.oarbot = OarbotKinematics()
        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        self.u4 = 0

        self.is_locked = False
        rospy.Timer(rospy.Duration(0.04),self.forward_kin)

    def callback(self, msg):
        self.inverse_kin(msg)

    def inverse_kin(self, msg):
        self.motor_lock.acquire()
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
        self.controller_f.send_command(cmds.DUAL_DRIVE, -self.u1, self.u2)
        self.controller_b.send_command(cmds.DUAL_DRIVE, self.u3, -self.u4)
        self.motor_lock.release()     
        
    def read_speed(self, controller, motor_number):
        succeeded = False
        while not succeeded:
            message = controller.read_value("?S", motor_number)
            # print(message)
            a = message.split('=')
            if (len(a) > 1):
                try:
                    float(a[1])
                    succeeded = True
                except ValueError:
                    print("ValueError! message recieved: ")
                    print(message)
                    #pass
        return float(a[1])/60*2*math.pi


    def forward_kin(self,event):
        self.motor_lock.acquire()
        u1a = -self.read_speed(self.controller_f, 1)
        u2a = self.read_speed(self.controller_f, 2)
        u3a = self.read_speed(self.controller_b, 1)
        u4a = -self.read_speed(self.controller_b, 2)
    
        self.vel_feedback.linear.x = -self.oarbot.r/4 * (u1a + u2a + u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.linear.y = self.oarbot.r/4 * (-u1a + u2a - u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.angular.z = self.oarbot.r/(4*(self.oarbot.l + self.oarbot.w)) * (-u1a + u2a + u3a - u4a) * \
        self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.motor_lock.release()
        self.vel_pub.publish(self.vel_feedback)

if __name__ == "__main__":
    oarbot = OarbotControl()
    rospy.spin()
