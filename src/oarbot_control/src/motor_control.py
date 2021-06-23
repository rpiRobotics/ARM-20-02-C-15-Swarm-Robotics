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
        rospy.init_node('oarbot_ctrl', anonymous=True)

        self.motor_cmd = MotorCmd()
        serial_front = rospy.get_param('~serial_front')
        serial_back = rospy.get_param('~serial_back')

        # connection to Roboteq robot controller
        self.controller_f = RoboteqHandler(debug_mode=False, exit_on_interrupt=False)
        self.controller_b = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
        self.connected_f = self.controller_f.connect(serial_front)
        self.connected_b = self.controller_b.connect(serial_back)
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
    
        self.motor_lock.release()

if __name__ == "__main__":
    oarbotControl_Motor = OarbotControl_Motor()
    rospy.spin()
