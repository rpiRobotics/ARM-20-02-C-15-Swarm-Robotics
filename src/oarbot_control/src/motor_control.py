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
        rospy.init_node('oarbot_ctrl_motor', anonymous=True)

        self.serial_front = rospy.get_param('~serial_front')
        self.serial_back = rospy.get_param('~serial_back')
        self.motor_command_topic_name = rospy.get_param('~motor_command_topic_name')
        self.motor_feedback_name = rospy.get_param('~motor_feedback_topic_name')

        rospy.Subscriber(self.motor_command_topic_name, MotorCmd, self.motor_cmd_callback, queue_size=1)
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
        with self.motor_lock:
            self.controller_f.send_command(cmds.DUAL_DRIVE, msg.v_fl, msg.v_fr)
            self.controller_b.send_command(cmds.DUAL_DRIVE, msg.v_rl, msg.v_rr)
          
        
    def read_speed(self, controller, motor_number):
        succeeded = False
        while not succeeded:
            message = controller.read_value(cmds.READ_SPEED, motor_number)
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
        return float(a[1])


    def motor_feedback(self,event):
        motor_feedback_msg = MotorCmd()

        with self.motor_lock:
            motor_feedback_msg.v_fl = self.read_speed(self.controller_f, 1)
            motor_feedback_msg.v_fr = self.read_speed(self.controller_f, 2)
            motor_feedback_msg.v_rl = self.read_speed(self.controller_b, 1)
            motor_feedback_msg.v_rr = self.read_speed(self.controller_b, 2)

        self.motor_feedback_pub.publish(motor_feedback_msg)

if __name__ == "__main__":
    oarbotControl_Motor = OarbotControl_Motor()
    rospy.spin()