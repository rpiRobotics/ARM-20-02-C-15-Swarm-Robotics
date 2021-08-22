#!/usr/bin/env python
import rospy
import time
import tf2_ros

from swarm_msgs.msg import State2D
from geometry_msgs.msg import TransformStamped
import tf_conversions

'''
robot_sim.py
Alex Elias

Simulates closed-loop behavior of robots
Input to each simulated robot is a State2D command
Output of each robot is a tf frame

Parameters:
    state_command_topic_names: List of State2D topic names
    tf_frame_names: List of tf frame names
'''

MAX_TIMESTEP = 0.1
K = 5


class Robot_sim:
    def __init__(self):
        rospy.init_node('robot_sim', anonymous=False)

        # Read in all the parameters
        state_command_topic_names = rospy.get_param('~state_command_topic_names')
        self.tf_frame_names = rospy.get_param('~tf_frame_names')

        self.N_robots = len(state_command_topic_names)

        # Initialize
        self.last_timestep_requests = {}

        # First index is i_robot
        # Second index is [x, y, theta]
        self.robot_states = x = [[0.0, 0.0, 0.0] for i in range(self.N_robots)]

        # Subscribe
        for i in range(self.N_robots):
            rospy.Subscriber(state_command_topic_names[i], State2D, self.just_state_command_callback, i, queue_size=1)

        # Publish
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Timer
        rospy.Timer(rospy.Duration(0.01), self.publish_frames_callback)


    def just_state_command_callback(self, data, i_robot):
        dt = self.get_timestep(i_robot)
        self.robot_states[i_robot][0] += dt * (data.twist.linear.x  + K *(data.pose.x     - self.robot_states[i_robot][0]))
        self.robot_states[i_robot][1] += dt * (data.twist.linear.y  + K *(data.pose.y     - self.robot_states[i_robot][1]))
        self.robot_states[i_robot][2] += dt * (data.twist.angular.z + K *(data.pose.theta - self.robot_states[i_robot][2]))

    def get_timestep(self, integrator_name):
        current_time = time.time()
        if integrator_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[integrator_name]
            self.last_timestep_requests[integrator_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[integrator_name] = current_time
            return 0.0

    def publish_frames_callback(self, event):
        for i in range(self.N_robots):
            msg = xyt2TF(self.robot_states[i], "map", self.tf_frame_names[i])
            self.tf_broadcaster.sendTransform(msg)

def xyt2TF(xyt, header_frame_id, child_frame_id):
    '''
    Converts a list [x y theta]
    into a tf2_msgs.msg.TFMessage message
    '''
    t = TransformStamped()

    t.header.frame_id = header_frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = xyt[0]
    t.transform.translation.y = xyt[1]
    t.transform.translation.z = 0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t

if __name__ == '__main__':
    robot_sim = Robot_sim()
    rospy.spin()