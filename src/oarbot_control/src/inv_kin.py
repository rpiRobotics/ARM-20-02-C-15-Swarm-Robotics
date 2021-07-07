#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from swarm_msgs.msg import MotorCmd
import math

class OarbotControl_InvKin():
    def __init__(self):
        rospy.init_node('oarbot_ctrl_inv_kin', anonymous=True)

        self.motor_command_name=rospy.get_param('~motor_command_topic_name')
        self.teleop_command_name=rospy.get_param('~republished_spacemouse')

        self.l_x        = rospy.get_param("~l_x") # meter
        self.l_y        = rospy.get_param("~l_y") # meter
        self.r          = rospy.get_param("~radius_wheel") # meter
        self.total_gear_ratio  = rospy.get_param("~total_gear_ratio") # rad/s
        self.skid_steer_mode = rospy.get_param("~skid_steer_mode", False) # boolean
 
        self.motor_cmd_pub = rospy.Publisher(self.motor_command_name, MotorCmd, queue_size=1)
        rospy.Subscriber(self.teleop_command_name, Twist, self.callback, queue_size=1)

    def callback(self, msg):
        self.inverse_kin(msg)

    def inverse_kin(self, msg):
        if self.skid_steer_mode:
            msg.linear.y = 0
            
        v_lin = msg.linear
        v_ang = msg.angular
        v_lin.x = -v_lin.x
        v_lin.y = -v_lin.y
        
        v_fl = 1/self.r * (v_lin.x + v_lin.y - (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of front right motor
        v_fr = 1/self.r * (v_lin.x - v_lin.y + (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of rear right motor
        v_rl = 1/self.r * (v_lin.x + v_lin.y + (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        # angular velocity of rear left motor
        v_rr = 1/self.r * (v_lin.x - v_lin.y - (self.l_x+self.l_y)*v_ang.z) * self.total_gear_ratio
        
        # Convert from rad/s to RPM
        v_fl = v_fl*60/(2*math.pi)
        v_fr = v_fr*60/(2*math.pi)
        v_rl = v_rl*60/(2*math.pi)
        v_rr = v_rr*60/(2*math.pi)

        # Generate and publish the MotorCmd message
        motor_cmd = MotorCmd()
        motor_cmd.v_fl = -v_fl
        motor_cmd.v_fr = v_fr
        motor_cmd.v_rl = v_rl
        motor_cmd.v_rr = -v_rr

        self.motor_cmd_pub.publish(motor_cmd)
        
if __name__ == "__main__":
    oarbotControl_InvKin = OarbotControl_InvKin()
    rospy.spin()