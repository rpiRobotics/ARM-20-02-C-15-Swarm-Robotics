#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from swarm_msgs.msg import MotorStatus, MotorCmd
from roboteq_handler import RoboteqHandler
import roboteq_commands as cmds
import threading

class OarbotControl_Motor():
    def __init__(self):
        self.motor_lock = threading.Lock()
        self.last_vel_lock = threading.Lock()
        rospy.init_node('oarbot_ctrl_motor', anonymous=True)

        self.serial_front = rospy.get_param('~serial_front')
        self.serial_back = rospy.get_param('~serial_back')
        self.motor_command_topic_name = rospy.get_param('~motor_command_topic_name')
        self.motor_feedback_name = rospy.get_param('~motor_feedback_topic_name')

        rospy.Subscriber(self.motor_command_topic_name, MotorCmd, self.motor_cmd_callback, queue_size=1)
        self.motor_feedback_pub = rospy.Publisher(self.motor_feedback_name, MotorCmd, queue_size=1)
        self.motor_feedback_pub = rospy.Publisher(self.motor_feedback_name, MotorCmd, queue_size=1)

        # connection to Roboteq motor controller
        self.connect_Roboteq_controller()
        
        rospy.Timer(rospy.Duration(0.04), self.motor_feedback)

    def connect_Roboteq_controller(self):
        self.controller_f = RoboteqHandler()
        self.controller_b = RoboteqHandler() 
        self.connected_f = self.controller_f.connect(self.serial_front)
        self.connected_b = self.controller_b.connect(self.serial_back)

    def motor_cmd_callback(self, msg):
        with self.last_vel_lock:
            self.motor_cmd_msg = msg
            self.velocity_command_sent = False

        
    def format_speed(self, speed_message):
        # Formats the speed message (RPM) obtained from roboteq driver into float
        try:
            rpm = speed_message.split('=')
            assert rpm[0] == 'S'or rpm[0] == 's' # To make sure that is a speed reading
            return float(rpm[1])
        except:
            rospy.logwarn("Improper motor speed message:" + speed_message)


    def motor_feedback(self,event):
        motor_feedback_msg = MotorCmd()

        with self.motor_lock:
            # Execute the motor velocities 
            with self.last_vel_lock:
                if self.velocity_command_sent:
                    self.controller_f.send_command(cmds.DUAL_DRIVE, 0.0, 0.0)
                    self.controller_b.send_command(cmds.DUAL_DRIVE, 0.0, 0.0)
                else:
                    self.controller_f.send_command(cmds.DUAL_DRIVE, self.motor_cmd_msg.v_fl, self.motor_cmd_msg.v_fr)
                    self.controller_b.send_command(cmds.DUAL_DRIVE, self.motor_cmd_msg.v_rl, self.motor_cmd_msg.v_rr)
                    self.velocity_command_sent = True

            # Read the executed motor velocities
            motor_feedback_msg.v_fl = self.format_speed(self.controller_f.read_value(cmds.READ_SPEED, 1))
            motor_feedback_msg.v_fr = self.format_speed(self.controller_f.read_value(cmds.READ_SPEED, 2))
            motor_feedback_msg.v_rl = self.format_speed(self.controller_b.read_value(cmds.READ_SPEED, 1))
            motor_feedback_msg.v_rr = self.format_speed(self.controller_b.read_value(cmds.READ_SPEED, 2))


        self.motor_feedback_pub.publish(motor_feedback_msg)

if __name__ == "__main__":
    oarbotControl_Motor = OarbotControl_Motor()
    rospy.spin()