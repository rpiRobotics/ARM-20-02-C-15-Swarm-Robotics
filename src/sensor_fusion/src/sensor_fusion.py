#!/usr/bin/env python
import numpy as np
import rospy
import threading
import time

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import tf_conversions # quaternion stuff

from std_msgs.msg import String # For UWB messages
from std_msgs.msg import Float32 # For UWB rmse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

# Local python files
from multilateration import *
from UWB_odom_kalman_filter import *
from uwb_parsing import *

'''
sensor_fusion.py
Alex Elias

Estimates robot position using UWB data and odometry
(Code assume tags are at same Z postion)

Parameters:
	velocity_feedback_type: "Odometry" or "Twist"
	velocity_feedback_topic_name: e.g. "ridgeback_velocity_controller/odom"
	uwb_front_topic_name: e.g. "uwb_serial_front" 
	uwb_back_topic_name: e.g. "uwb_serial_back"

	tf_frame_name_uwb:   			e.g. "OARBOT_1_base_link_UWB"
	tf_frame_name_fused: 			e.g. "OARBOT_1_base_link_RPI_fusion"
	position_feedback_topic_name: 	e.g. "oarbot_1_position"

	tag_loc_front_x: X location in m of front tag (towards front of robot)
	tag_loc_front_y: Y location in m of front tag (towards left of robot)

	tag_loc_back_x: X location in m of back tag (towards front of robot)
	tag_loc_back_y: Y location in m of back tag (towards left of robot)	
'''


# Max time (in seconds) to consider UWB readings to happen simulataneously
# Note that UWB readings happen at 10 Hz
UWB_TIMEOUT = 0.05

DEBUG_UWB = False

class Fusion:
	def __init__(self):
		rospy.init_node('sensor_fusion', anonymous=False)

		# Read in all parameters
		self.feedback_is_Odometry = ("Odometry" == rospy.get_param('~velocity_feedback_type'))

		self.velocity_feedback_topic_name = rospy.get_param('~velocity_feedback_topic_name')
		self.uwb_front_topic_name = rospy.get_param('~uwb_front_topic_name')
		self.uwb_back_topic_name = rospy.get_param('~uwb_back_topic_name')

		uwb_front_id = rospy.get_param('~uwb_front_id')
		uwb_back_id = rospy.get_param('~uwb_back_id')

		self.tf_frame_name_uwb = rospy.get_param('~tf_frame_name_uwb')
		self.tf_frame_name_fused = rospy.get_param('~tf_frame_name_fused')
		self.position_feedback_topic_name = rospy.get_param('~position_feedback_topic_name')

		tag_loc_front_x = rospy.get_param('~tag_loc_front_x')
		tag_loc_front_y = rospy.get_param('~tag_loc_front_y')
		self.tag_loc_front = np.array([[tag_loc_front_x],[tag_loc_front_y]])

		tag_loc_back_x = rospy.get_param('~tag_loc_back_x')
		tag_loc_back_y = rospy.get_param('~tag_loc_back_y')
		self.tag_loc_back = np.array([[tag_loc_back_x],[tag_loc_back_y]])

		uwb_meas_std = rospy.get_param('~uwb_meas_std')
		odom_meas_std = rospy.get_param('~odom_meas_std')
		process_pos_std = rospy.get_param('~process_pos_std')
		process_vel_std = rospy.get_param('~process_vel_std')

		self.antenna_offsets = rospy.get_param('~antenna_offsets')
		self.front_offset = self.antenna_offsets[uwb_front_id]
		self.back_offset = self.antenna_offsets[uwb_back_id]

		self.tag_z_height = rospy.get_param('~tag_z_height',None)

		# Kalman filter state, covariance, and time
		self.state = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]) # x,y,theta,x_dot,y_dot,theta_dot
		self.cov   = 100.0**2 * np.eye(6)
		self.kalman_time = time.time()

		self.kalman_lock = threading.Lock()

		# timestamp of reading
		self.front_t = 0.
		self.back_t = 0.

		# Matrix of anchor positions
		self.front_anchors = 0.
		self.back_anchors = 0.

		# Vector of distances to each anchor
		self.front_dists = 0.
		self.back_dists = 0.

		# IDs corresponding to anchors
		self.front_ids = 0.
		self.back_ids = 0.

		# Initialize Extended Kalman Filter Object
		self.ekf = EKF(uwb_meas_std, odom_meas_std, process_pos_std, process_vel_std)

		# Publish position
		self.pos_pub = rospy.Publisher(self.position_feedback_topic_name, Pose2D, queue_size=1)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

		self.rmse_pub = rospy.Publisher(self.position_feedback_topic_name + '_UWB_RMSE', Float32, queue_size=10)
		self.odom_pub = rospy.Publisher(self.position_feedback_topic_name + '_odom_kalman', Odometry, queue_size=10) # Only for data analysis
		self.odom_seq = 0; # Sequence number for odometry publishing

		# Subscribe to UWB tags
		rospy.Subscriber(self.uwb_front_topic_name, String, self.uwb_serial_front_callback, queue_size=1)
		rospy.Subscriber(self.uwb_back_topic_name, String, self.uwb_serial_back_callback, queue_size=1)

		# Subscribe to velocity / odometry
		if self.feedback_is_Odometry:
			rospy.Subscriber(
				self.velocity_feedback_topic_name, Odometry, self.odom_callback, queue_size=1)
		else: # Feedback is Velocity
			rospy.Subscriber(
				self.velocity_feedback_topic_name, Twist, self.odom_callback, queue_size=1)

	def uwb_serial_front_callback(self, data):
		if DEBUG_UWB:
			rospy.logwarn("uwb_serial_front_callback")
		valid, anchor_mat, dists, ids = parse_lec_line(data.data)
		if not valid:
			rospy.logwarn("NOT VALID")
			rospy.logwarn(str(data))
			return

		self.front_t = time.time()
		self.front_anchors = anchor_mat
		self.front_dists = dists 
		self.front_ids = ids

		if DEBUG_UWB:
			rospy.logwarn("self.front_t - self.back_t")
			rospy.logwarn(str(self.front_t - self.back_t))
		if (self.front_t - self.back_t) < UWB_TIMEOUT:
			self.combine_uwb_readings()

	def uwb_serial_back_callback(self, data):
		if DEBUG_UWB:
			rospy.logwarn("uwb_serial_back_callback")
		valid, anchor_mat, dists, ids = parse_lec_line(data.data)
		if not valid:
			rospy.logwarn("NOT VALID")
			rospy.logwarn(str(data))
			return

		self.back_t = time.time();
		self.back_anchors = anchor_mat
		self.back_dists = dists
		self.back_ids = ids

		if DEBUG_UWB:
			rospy.logwarn("self.back_t - self.front_t")
			rospy.logwarn(str(self.back_t - self.front_t))
		if (self.back_t - self.front_t) < UWB_TIMEOUT:
			self.combine_uwb_readings()

	def combine_uwb_readings(self):
		if DEBUG_UWB:
			rospy.logwarn("Combining UWB readings!")
		# Kalman filter
		with self.kalman_lock as lock:
			dt = max(self.front_t, self.back_t) - self.kalman_time
			if dt < 0:	
				# # lock.release()
				rospy.logwarn("Dropping UWB reading | dt = " + str(dt))
				return
			if dt > 0.25:
				# rospy.logwarn("Limiting UWB timestep to 1 | dt = " + str(dt))
				rospy.logwarn("UWB timestep is greater than 0.25 s | dt = " + str(dt))
				dt = 0.

			# Ignore reading if there are less than 8 readings
			if self.front_dists.size + self.back_dists.size < 8:
				# # # lock.release()
				rospy.logwarn("Dropping UWB reading | number of readings = " + str(self.front_dists.size + self.back_dists.size) + " which is less than 8" )
				return

			# Correct the readings with the offsets
			front_anchor_offsets = np.array([[self.antenna_offsets[i] for i in self.front_ids]]).T
			back_anchor_offsets  = np.array([[self.antenna_offsets[i] for i in self.back_ids]]).T
			
			corrected_front_dists = self.front_dists + self.front_offset + front_anchor_offsets
			corrected_back_dists = self.back_dists + self.back_offset + back_anchor_offsets

			# Multilateration
			uwb_pos, rmse = tag_pair_min_z(self.front_anchors, self.back_anchors,
			corrected_front_dists, corrected_back_dists, self.tag_loc_front, self.tag_loc_back, self.tag_z_height)

			# Ignore reading if rmse is high than ... meters
			if rmse > 0.15:
				# # # lock.release()
				rospy.logwarn("Dropping UWB reading | rmse = " + str(rmse) + " is too high" )
				return

			self.kalman_time =  max(self.front_t, self.back_t)

			self.state, self.cov, self.kalman_pos = self.ekf.EKF_UWB(self.state, self.cov, dt, uwb_pos[[0,1,3],np.newaxis], rmse)

		# Publish
		tf_kalman = xyt2TF(self.kalman_pos, "map", self.tf_frame_name_fused)
		self.tf_broadcaster.sendTransform(tf_kalman)
		self.rmse_pub.publish(data=rmse)

		tf_uwb = xyzt2TF(uwb_pos, "map", self.tf_frame_name_uwb)
		self.tf_broadcaster.sendTransform(tf_uwb)
		self.publish_position()

		# ###
		# import csv
		# import os 
		# current_dir = os.path.expanduser("~")
		# location = os.path.join(current_dir,'uwb_multilateration_data.csv')
		
		# with open(location, mode='a') as data_file:
		# 	data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		# 	data_writer.writerow(uwb_pos.flatten().tolist())

		# location = os.path.join(current_dir,'uwb_rmse_data.csv')
		# with open(location, mode='a') as data_file:
		# 	data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		# 	data_writer.writerow([rmse])

		# location = os.path.join(current_dir,'uwb_num_dist_readings.csv')
		# with open(location, mode='a') as data_file:
		# 	data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		# 	data_writer.writerow([self.front_dists.size + self.back_dists.size])



	def odom_callback(self, data):
		# Assemble the measurement vector
		if self.feedback_is_Odometry:
			x_d = data.twist.twist.linear.x
			y_d = data.twist.twist.linear.y
			theta_d = data.twist.twist.angular.z
		else: # Feedback is Velocity
			x_d = data.linear.x
			y_d = data.linear.y
			theta_d = data.angular.z
		meas = np.array([[x_d],[y_d],[theta_d]])

		# Kalman filter
		with self.kalman_lock as lock:
			t = time.time()
			dt = t - self.kalman_time
			if dt < 0:	
				# # lock.release()
				rospy.logwarn("Dropping odom reading | dt = " + str(dt))
				return
			if dt > 0.25:
				# rospy.logwarn("Limiting odom timestep to 1 | dt = " + str(dt))
				rospy.logwarn("Odom timestep is greater than 0.25 s | dt = " + str(dt))
				dt = 0.
				# self.kalman_time = t
				# return
			
			self.kalman_time = t
			self.state, self.cov, self.kalman_pos = self.ekf.EKF_odom(self.state, self.cov, dt, meas)

		# Publish
		tf_kalman = xyt2TF(self.kalman_pos, "map", self.tf_frame_name_fused);
		self.tf_broadcaster.sendTransform(tf_kalman)
		self.publish_position()

		
	def publish_position(self):
		# Pose2D message for the control loop
		pos_msg = Pose2D()
		pos_msg.x = self.state[0][0]
		pos_msg.y = self.state[1][0]
		pos_msg.theta = self.state[2][0]
		self.pos_pub.publish(pos_msg)

		# Odometry messaage for data analysis
		# Twist is in world frame, not robot frame
		odom_msg = Odometry()

		self.odom_seq =+ 1
		odom_msg.header.seq = self.odom_seq
		odom_msg.header.stamp = rospy.Time.now()
		odom_msg.header.frame_id = 'map'

		odom_msg.child_frame_id = self.tf_frame_name_fused

		odom_msg.pose.pose.position.x = self.state[0][0]
		rospy.logwarn(str(self.state))
		odom_msg.pose.pose.position.y = self.state[1][0]
		# odom_msg.pose.pose.position.z

		q = tf_conversions.transformations.quaternion_from_euler(0, 0,self.state[2][0])
		odom_msg.pose.pose.orientation.x = q[0]
		odom_msg.pose.pose.orientation.y = q[1]
		odom_msg.pose.pose.orientation.z = q[2]
		odom_msg.pose.pose.orientation.w = q[3]

		# Instead of (x, y, z, rot_x, rot_y, rot_z)
		# Use (x, y, theta, x_dot, y_dot, theta_dot)
		odom_msg.pose.covariance = self.cov.flatten().tolist()

		odom_msg.twist.twist.linear.x = self.state[3][0]
		odom_msg.twist.twist.linear.y = self.state[4][0]
		#odom_msg.twist.twist.linear.z

		#odom_msg.twist.twist.angular.x
		#odom_msg.twist.twist.angular.x
		odom_msg.twist.twist.angular.z = self.state[5][0]

		#odom_msg.twist.twist.angular.covariance

		self.odom_pub.publish(odom_msg)



def xyzt2TF(xyzt, header_frame_id, child_frame_id):
	xyzt = xyzt.flatten()
	'''
	Converts a numpy vector [x; y; z; theta]
	into a tf2_msgs.msg.TFMessage message
	'''
	t = geometry_msgs.msg.TransformStamped()

	t.header.frame_id = header_frame_id
	t.header.stamp = rospy.Time.now()
	t.child_frame_id = child_frame_id
	t.transform.translation.x = xyzt[0]
	t.transform.translation.y = xyzt[1]
	t.transform.translation.z = xyzt[2]

	q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyzt[3])
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	return t

def xyt2TF(xyt, header_frame_id, child_frame_id):
	xyt = xyt.flatten()
	'''
	Converts a numpy vector [x; y; theta]
	into a tf2_msgs.msg.TFMessage message
	'''
	t = geometry_msgs.msg.TransformStamped()

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
	if DEBUG_UWB:
			rospy.logwarn("Starting UWB debug mode on sensor fusion node!")

	Fusion()
	rospy.spin()