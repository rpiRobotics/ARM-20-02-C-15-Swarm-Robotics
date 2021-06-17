#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from oarbot_control.msg import Twist2D, MotorStatus, MotorCmd
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
        #self.rate = rospy.Rate(10) # 10hz
        #self.vel_feedback = Twist2D() # velocity feedback
        self.vel_feedback = Twist()
        self.motor_cmd = MotorCmd() # motor velocity command
        # actual velocity publisher
        self.vel_feedback_name=rospy.get_param('~velocity_feedback_topic_name')
        self.motor_command_name=rospy.get_param('~motor_command_topic_name')
        self.teleop_command_name=rospy.get_param('~republished_spacemouse')
        serial_front = rospy.get_param('~serial_front')
        serial_back = rospy.get_param('~serial_back')
        #self.vel_feedback_name='oarbot1/vel_feedback'
        #self.motor_command_name='oarbot1/motor_command'
        #self.teleop_command_name='spacenav/twist/repub2'
        self.vel_pub = rospy.Publisher(self.vel_feedback_name, Twist, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)
        # connection to Roboteq robot controller
        self.controller_f = RoboteqHandler(debug_mode=False, exit_on_interrupt=False)
        self.controller_b = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
        self.connected_f = self.controller_f.connect(serial_front)
        self.connected_b = self.controller_b.connect(serial_back)
        rospy.Subscriber(self.teleop_command_name, Twist, self.callback, queue_size=1)
        #rospy.Subscriber("oarbot_1_vel_cmd", Twist, self.callback)
        self.oarbot = OarbotKinematics()
        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        self.u4 = 0

        self.is_locked = False
        rospy.Timer(rospy.Duration(0.04),self.forward_kin)

    def callback(self, msg):
        v = msg.linear
        #print '------- commanded --------'
        w = msg.angular
        #print v.x, v.y, w.z
        #print self.moto
        #if self.motor_lock.locked():
        #    return
        #self.motor_lock.acquire()
        #print self.motor_lock.locked()
        #t1 = time.time()
        # do the motor control here
        self.inverse_kin(msg)
        #elf.forward_kin()

        #self.motor_lock.release()
        #t2 = time.time()
        #print t2-t1

    def inverse_kin(self, msg):
        #if self.connected_f and self.connected_b:
        self.motor_lock.acquire()
        v_lin = msg.linear
        v_ang = msg.angular
        #print(self.oarbot.chain_drive_ratio)
        #print('hello')
        # angular velocity of front left motor
        v_lin.x = -v_lin.x
        v_lin.y = -v_lin.y
        
        self.u1 = 1/self.oarbot.r * (v_lin.x + v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of front right motor
        self.u2 = 1/self.oarbot.r * (v_lin.x - v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of rear right motor
        self.u3 = 1/self.oarbot.r * (v_lin.x + v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        # angular velocity of rear left motor
        self.u4 = 1/self.oarbot.r * (v_lin.x - v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.chain_drive_ratio * 1/self.oarbot.gear_ratio
        
        # convert from rad/s to RPM
        self.u1 = self.u1*60/(2*math.pi)
        self.u2 = self.u2*60/(2*math.pi)
        self.u3 = self.u3*60/(2*math.pi)
        self.u4 = self.u4*60/(2*math.pi)
        # print '-----------commanded----------------'
        # print -self.u1, self.u2, self.u3, -self.u4

        # emergency stop if v too large
        # if abs(self.u1) > 500 or abs(self.u2) > 500 or abs(self.u3) > 500 or abs(self.u4) > 500:
        #    self.u1 = self.u2 = self.u3 = self.u4 = 0
        #TODO: check units
        self.motor_cmd.v_fl = -self.u1
        self.motor_cmd.v_fr = self.u2
        self.motor_cmd.v_rl = self.u3
        self.motor_cmd.v_rr = -self.u4

        self.motor_cmd_pub.publish(self.motor_cmd)
        # front motors velocity control
        #print("Left:")
        #print((-self.u1*2.5, self.u2*2.))
        self.controller_f.send_command(cmds.DUAL_DRIVE, -self.u1, self.u2)
        # rear motors velocity control
        #print("Right:")
        #print((self.u3*2.5, -self.u4*2.5))
        self.controller_b.send_command(cmds.DUAL_DRIVE, self.u3, -self.u4)
        self.motor_lock.release()
        #print -self.u1*2.5, self.u2*2.5, self.u3*2.5, -self.u4*2.5
        #print("inverse end")
        # print self.controller_f.read_value(cmds.REAL_BL_MOTOR_RPM)            
        
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


    # using forward kinematics to compute actual v
    def forward_kin(self,event):
        #TODO: read actual velocity from roboteq
        #print("fwrd kin start")
        #print '------------- feedback -----------'
        #v1 = self.controller_f.read_value(cmds.READ_BL_MOTOR_RPM, 1)
        
        # v1 = self.controller_f.read_value("?S", 1)
        # print v1
        # a = v1.split('=')
        # if (len(a) > 1):
        # #print a[1]
        #     u1a = -float(a[1])/60*2*math.pi
        # else:
        #     u1a = -self.u1/60*2*math.pi
        self.motor_lock.acquire()
        u1a = -self.read_speed(self.controller_f, 1)
       
        # v2 = self.controller_f.read_value("?S", 2)
        # print v2
        # a = v2.split('=')
        # if (len(a) > 1):
        # #print a[1]
        #     u2a = float(a[1])/60*2*math.pi
        # else:
        #     u2a = self.u2/60*2*math.pi

        u2a = self.read_speed(self.controller_f, 2)

        # #print('fwrd kin forward pass')
        # v3 = self.controller_b.read_value("?S", 1)
        # print v3
        # a = v3.split('=')
        # if (len(a) > 1):
        # #print a[1]
        #     u3a = float(a[1])/60*2*math.pi
        # else:
        #     u3a = self.u3/60*2*math.pi

        u3a = self.read_speed(self.controller_b, 1)

        # v4 = self.controller_b.read_value("?S", 2)
        # print v4
        # a = v4.split('=')
        # #print a
        # if (len(a) > 1):
        # #print a[1]
        #     u4a = -float(a[1])/60*2*math.pi
        # else:
        #     u4a = -self.u4/60*2*math.pi

        u4a = -self.read_speed(self.controller_b, 2)
    
        #print '----------- feedback ----------------'
        self.vel_feedback.linear.x = -self.oarbot.r/4 * (u1a + u2a + u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.linear.y = self.oarbot.r/4 * (-u1a + u2a - u3a + u4a) * self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        self.vel_feedback.angular.z = self.oarbot.r/(4*(self.oarbot.l + self.oarbot.w)) * (-u1a + u2a + u3a - u4a) * \
        self.oarbot.chain_drive_ratio * self.oarbot.gear_ratio
        #self.vel_feedback.linear.x = -self.vel_feedback.linear.x
        #self.vel_feedback.linear.y = -self.vel_feedback.linear.y
        self.motor_lock.release()
        self.vel_pub.publish(self.vel_feedback)
        #print self.vel_feedback.linear.x, self.vel_feedback.linear.y
        #print u1a, u2a, u3a, u4a
            #print("fwd kin end")
        #print -self.vel_feedback.linear.x, -self.vel_feedback.linear.y, self.vel_feedback.angular.z
        #TODO: add motor status feedback

    def run(self):
        while not rospy.is_shutdown():
            if self.connected_f and self.connected_b:
                #print('hello')
                #self.forward_kin()
                continue
            #else:
            #    print("hi")
          


if __name__ == "__main__":
    oarbot = OarbotControl()
    rospy.spin()
    #oarbot.run()
