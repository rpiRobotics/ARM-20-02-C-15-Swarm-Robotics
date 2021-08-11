#!/usr/bin/env python
import rospy

# import State2D.msg
from swarm_msgs.msg import State2D
from geometry_msgs.msg import Twist, PoseStamped
import geometry_msgs.msg

import tf2_ros
import tf2_msgs.msg
from std_msgs.msg import Bool
import tf_conversions # quaternion stuff

from safe_swarm_controller import *

import numpy as np

import time


'''
swarm_control.py
Alex Elias

Commands robot states (position and velocity) to achieve rigid-body motion

Parameters:

    desired_swarm_vel_topic_name:
    just_swarm_frame_vel_input_topic_name:

    N_robots: Numer of robots to control

    theta_scale: How much we weigh theta vs x,y in choosing safe swarm velocity
        Higher theta_scale ==> Swarm velocity follows angle commands closer
        (But doesn't follow XY commands as closely)

    Repeat for all N robots:
        just_robot_vel_input_topic_name_0:
        state_publish_topic_name_0: State2D command for robot 0
        tf_frame_name_0: Frame name (for rviz) for robot 0
        vel_lim_x_0: Velocity limit in X for robot 0
        vel_lim_y_0: Velocity limit in Y for robot 0
        vel_lim_theta_0: Velocity limit in theta for robot 0
        acc_lim_x_0: Acceleration limit in X for robot 0
        acc_lim_y_0: Acceleration limit in Y for robot 0
        acc_lim_theta_0: Acceleration limit in theta for robot 0

'''

# Velocity commands will only be considered if they are spaced closer than MAX_TIMESTEP
MAX_TIMESTEP = 0.1

class Swarm_Control:
	def __init__(self):
		rospy.init_node('swarm_controller', anonymous=False)

		# Read in all the parameters
		desired_swarm_vel_topic_name = rospy.get_param('~desired_swarm_vel_topic_name')
		just_swarm_frame_vel_input_topic_name = rospy.get_param('~just_swarm_frame_vel_input_topic_name')

		self.N_robots = rospy.get_param('~N_robots')
		self.theta_scale = rospy.get_param('~theta_scale')

		just_robot_vel_input_topic_names = ['']*self.N_robots
		state_publish_topic_names = ['']*self.N_robots
		self.tf_frame_names = ['']*self.N_robots
		self.vel_pubs = [0]*self.N_robots

		self.v_max = np.zeros((3, self.N_robots))
		self.a_max = np.zeros((3, self.N_robots))

		for i in range(self.N_robots):
			just_robot_vel_input_topic_names[i] = rospy.get_param('~just_robot_vel_input_topic_name_' + str(i))
			state_publish_topic_names[i] = rospy.get_param('~state_publish_topic_name_' + str(i))
			self.tf_frame_names[i] = rospy.get_param('~tf_frame_name_' + str(i))

			self.v_max[0, i] = rospy.get_param('~vel_lim_x_' 	 + str(i))
			self.v_max[1, i] = rospy.get_param('~vel_lim_y_' 	 + str(i))
			self.v_max[2, i] = rospy.get_param('~vel_lim_theta_' + str(i))
			self.a_max[0, i] = rospy.get_param('~acc_lim_x_' 	 + str(i))
			self.a_max[1, i] = rospy.get_param('~acc_lim_y_'	 + str(i))
			self.a_max[2, i] = rospy.get_param('~acc_lim_theta_' + str(i))

		# Subscribe
		sync_frame_topic=rospy.get_param('~frame_sync_topic_name')
		rospy.Subscriber(desired_swarm_vel_topic_name, Twist, self.desired_swarm_velocity_callback, queue_size=1)
		rospy.Subscriber(just_swarm_frame_vel_input_topic_name, Twist, self.just_swarm_frame_velocity_callback, queue_size=1)
		rospy.Subscriber(sync_frame_topic, PoseStamped, self.frame_changer_callback, queue_size=20)
		for i in range(self.N_robots):
			rospy.Subscriber(just_robot_vel_input_topic_names[i], Twist, self.just_robot_velocity_callback, i, queue_size=1)

		# Publish
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()
		for i in range(self.N_robots):
			self.vel_pubs[i] = rospy.Publisher(state_publish_topic_names[i], State2D, queue_size=1)

		# Initialize local variables to keep track of swarm/robot frames
		self.swarm_xyt = np.zeros((3,1))
		self.robots_xyt = np.zeros((3, self.N_robots))
		self.robots_last_velocities = np.zeros((3, self.N_robots))
		self.last_timestep_requests = {}
		self.v_robots_prev = np.zeros((3,self.N_robots))

		# TF publish loop
		rate = rospy.Rate(30) # 30hz
 		while not rospy.is_shutdown():
 			self.publish_tf_frames()
 			rate.sleep()

	def frame_changer_callback(self,data):
		if(data.header.frame_id in self.tf_frame_names):
			array_position=self.tf_frame_names.index(data.header.frame_id)
			orientations= [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
			(roll,pitch,yaw)=tf_conversions.transformations.euler_from_quaternion(orientations)
			rospy.logwarn(np.array2string(self.robots_xyt))
			self.robots_xyt[0,array_position]=data.pose.position.x
			self.robots_xyt[1,array_position]=data.pose.position.y
			self.robots_xyt[2,array_position]=yaw

	def desired_swarm_velocity_callback(self, data):
		dt = self.get_timestep("desired_swarm_velocity")
		if dt == 0: # Exceeded MAX_TIMESTEP
			return

		v_desired = np.zeros((3,1))
		v_desired[0, 0] = data.linear.x
		v_desired[1, 0] = data.linear.y
		v_desired[2, 0] = data.angular.z

		p_i_mat = self.robots_xyt[0:1+1,:]
		theta_vec = self.robots_xyt[[2],:]
		#print(p_i_mat)
		#print(theta_vec)
		v_i_world, v_i_rob, xyt_i, v, xyt_swarm_next = safe_motion_controller(
			v_desired, self.theta_scale, p_i_mat, theta_vec,
			self.v_max, self.a_max, dt, self.N_robots, self.v_robots_prev, self.swarm_xyt)

		# Don't update self.robots_xyt, since that's in the swarm frame
		self.v_robots_prev = v_i_rob;
		self.swarm_xyt = xyt_swarm_next;

		# Send desired state to each robot
		for i in range(self.N_robots):
			s = State2D();
			s.pose.x = xyt_i[0,i]
			s.pose.y = xyt_i[1,i]
			s.pose.theta = xyt_i[2,i]
			s.twist.linear.x = v_i_world[0,i]
			s.twist.linear.y = v_i_world[1,i]
			s.twist.angular.z = v_i_world[2,i]
			self.vel_pubs[i].publish(s)

	def just_swarm_frame_velocity_callback(self, data):
		'''
		Move the position of the swarm frame without moving the robots
		i.e. move the swarm frame, and move the robots
			w.r.t. the swarm frame in the opposite direction
		'''
		dt = self.get_timestep("desired_swarm_frame_velocity")

		qd_world = np.zeros((3,1))
		qd_world[0, 0] = data.linear.x
		qd_world[1, 0] = data.linear.y
		qd_world[2, 0] = data.angular.z

		qd_swarm = rot_mat_3d(-self.swarm_xyt[2, 0]).dot(qd_world)

		q_world_delta = dt * qd_world
		q_swarm_delta = dt * qd_swarm

		self.swarm_xyt = self.swarm_xyt + q_world_delta

		#  np.diag([1., 1., 0.]).
		self.robots_xyt = rot_mat_3d(-q_swarm_delta[2,0]).dot(self.robots_xyt  - q_swarm_delta )

	def just_robot_velocity_callback(self, data, i_robot):
		# Move the position of robot i_robot in the world frame
		dt = self.get_timestep("just_robot_velocty_"+str(i_robot))

		qd_world = np.zeros((3,1))
		qd_world[0, 0] = data.linear.x
		qd_world[1, 0] = data.linear.y
		qd_world[2, 0] = data.angular.z

		# robots_xyt is in the swarm frame
		qd_swarm = rot_mat_3d(-self.swarm_xyt[2]).dot(qd_world)
		self.robots_xyt[:,i_robot] = self.robots_xyt[:,i_robot] + dt * qd_swarm.flatten()
		#print(self.robots_xyt)


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

	
	def publish_tf_frames(self):
		tf_swarm_frame = xyt2TF(self.swarm_xyt, "map", "swarm_frame")
		self.tf_broadcaster.sendTransform(tf_swarm_frame)

		for i in range(self.N_robots):
			tf_robot_i = xyt2TF(self.robots_xyt[:,i], "swarm_frame", self.tf_frame_names[i])
			self.tf_broadcaster.sendTransform(tf_robot_i)
            
def rot_mat_3d(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    
def wrapToPi(a):
    '''
    Wraps angle to [-pi,pi)
    '''
    return ((a+np.pi) % (2*np.pi))-np.pi

def xyt2TF(xyt, header_frame_id, child_frame_id):
    '''
    Converts a numpy vector [x; y; theta]
    into a tf2_msgs.msg.TFMessage message
    '''
    xyt = xyt.flatten()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = header_frame_id
    #t.header.stamp = ros_time #rospy.Time.now()
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
    Swarm_Control()
    rospy.spin()
