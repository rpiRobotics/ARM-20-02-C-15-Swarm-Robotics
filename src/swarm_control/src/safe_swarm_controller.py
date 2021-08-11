import numpy as np
from quadprog import solve_qp
import rospy

def safe_motion_controller(v_desired, theta_scale, p_i_mat, theta_vec,
	v_max, a_max, delta_t, N, v_i_prev, xyt_swarm):
	'''
	Minimize difference between desired and commanded swarm velocity
	Constraints: max velocity and accel of each robot

	Inputs:
		v_desired: Desired swarm velocity
			[x_dot; y_dot; theta_dot] (world frame)
		theta_scale:
			How much we weigh theta vs x,y in choosing safe swarm velocity
			Higher THETA_SCALE ==> Swarm velocity follows angle commands closer
			(But doesn't follow XY commands as closely)
		p_i_mat:
			Position of each robot in swarm frame
		theta_vec:
			Angle of each robot w.r.t. swarm frame
		v_max: Safe velocity will follow |v_i| < v_max (in robot frame)
		a_max: Safe velocity will follow |acceleration_i| < a_max (in robot frame)
		delta_t: Timestep
		N: Number of robots
		v_i_prev: Velcities during last iteration (for acceleration constraint)
		xyt_swarm: Current swarm position [x; y; theta]
	Outputs:
		v_i: Matrix, each col is safe velocity of each robot (world frame)
		xyt_i: v_i_prev + v_i*delta_t (next commanded position)
		v: Safe swarm velocity [x_dot; y_dot; theta_dot] (world frame)
		xyt_swarm_next: Next safe swarm position [x; y; theta]
	'''

	# Find the safe swarm velocity

	lsq_param_C = np.diag([1., 1., theta_scale])
	lsq_param_d = lsq_param_C.dot(v_desired)

	lsq_param_A = np.zeros((2*3*N, 3))
	for i in range(N):
		index_pos = range(6*i  ,6*i+3) # e.g. [0 1 2]
		index_neg = range(6*i+3,6*i+6) # e.g. [3 4 5]
		# Max velocity/accel constraints are wrt (stationary) robot frame
		p_i_world_frame = rot_mat(xyt_swarm[2][0]).dot(p_i_mat[:,[i]])
		theta_i_world_frame = xyt_swarm[2][0]+theta_vec[0][i]
		J = robot_jacobian(p_i_world_frame, theta_i_world_frame)
		lsq_param_A[index_pos, :] = J
		lsq_param_A[index_neg, :] = -J

	lsq_param_b = np.zeros((2*3*N, 1))
	for i in range(N):
		index_pos = range(6*i  ,6*i+3) # e.g. [0 1 2]
		index_neg = range(6*i+3,6*i+6) # e.g. [3 4 5]
		lsq_param_b[index_pos, :] = np.minimum(
			v_max[:,[i]], v_i_prev[:,[i]]+a_max[:,[i]]*delta_t)
		lsq_param_b[index_neg, :] = np.minimum(
			v_max[:,[i]], -v_i_prev[:,[i]]+a_max[:,[i]]*delta_t)

	try:
		v = lsqlin(lsq_param_C, lsq_param_d, lsq_param_A, lsq_param_b)
	except ValueError as e:
		rospy.logerr(str(e) + "\r\nC: " + str(lsq_param_C) + "\r\nd: " + str(lsq_param_d) + "\r\nA: " + str(lsq_param_A) + "\r\nb: " + str(lsq_param_b))

	# Find the velocity of each robot
	v_i_world = np.zeros((3,N))
	v_i_rob= np.zeros((3,N))
	for i in range(N):
		p_i_world_frame = rot_mat(xyt_swarm[2][0]).dot(p_i_mat[:,[i]])
		theta_i_world_frame = xyt_swarm[2][0]+theta_vec[0][i]
		J_rob = robot_jacobian(p_i_world_frame, theta_i_world_frame)
		J_world = robot_jacobian(p_i_world_frame, 0)
		#v_i_robot = J.dot(v)

		#v_i_world = v_i_robot
		#v_i_world[1:2+1] = rot_mat(theta_i_world_frame).dot(v_i_robot[1:2+1])

		#v_i[:,[i]] = #v_i_world	
		v_i_world[:,[i]] = J_world.dot(v)
		v_i_rob[:,[i]] = J_rob.dot(v)

	# Find the position of the swarm and each robot
	# Careful to find robot postion with forward kinematics, NOT integration
	xyt_swarm_next = xyt_swarm + v*delta_t

	xyt_i = np.zeros((3,N))
	for i in range(N):
		p_i_world_frame = rot_mat(xyt_swarm_next[2][0]).dot(p_i_mat[:,[i]])
		#theta_i_world_frame = xyt_swarm_next[2][0]+theta_vec[0][i]
		theta_i_world_frame = theta_vec[0][i]
		xyt_i_world_frame = np.block([[p_i_world_frame],[theta_i_world_frame]])
		# print('**********')
		# print(xyt_i[:,[i]])
		# print(xyt_swarm_next)
		# print(xyt_swarm_next)
		xyt_i[:,[i]] = xyt_swarm_next + xyt_i_world_frame
	
	# Return the results
	return v_i_world, v_i_rob, xyt_i, v, xyt_swarm_next

def robot_jacobian(p, theta):
	# Theta is the angle to rotate from the velocity frame to the final frame
	# p and v are in the same frame
	return np.block([
		[rot_mat(-theta),
		rot_mat(-theta).dot(np.array([[0., -1.], [1., 0.]])).dot(p) ],
		[0., 0., 1.]
		])

def rot_mat(theta):
	c, s = np.cos(theta), np.sin(theta)
	return np.array([[c, -s], [s, c]])

def lsqlin(C, d, A, b):
	'''
	Solves linear least-squares problem (using quadprog)
	TODO: we can also have equality constraints
	min_x 1/2  || Cx-d ||_2^2
	s.t. Ax <= b

	Quadprog need the following formulation
	min_x 1/2 x'Gx - a'x
	s.t. C'x >= b
	'''

	qp_G = C.T.dot(C)
	qp_a = C.T.dot(d).flatten()
	qp_C = -A.T
	qp_b = -b.flatten()

	#print(C)
	#print(d)

	min_x = solve_qp(qp_G, qp_a, qp_C, qp_b)[0]
	return min_x[:,np.newaxis]

def test_safe_motion_controller():
	#print(rot_mat_2d(0))
	#print(robot_jacobian(np.array([[1.],[0.]]), 0.))
	
	v_desired = np.array([[1.],[1.],[1.]]) 
	theta_scale = 1.

	P_1 = np.array([[0.],[2.]])
	P_2 = np.array([[0.],[-2.]])
	p_i_mat = np.block([P_1, P_2])

	theta_vec = np.array([[0., 0.]])

	v_max = np.array([[1., 1.],
				      [1., 1.],
					  [1., 1.]])
	a_max = np.array([[1., 1.],
				      [1., 1.],
					  [1., 1.]])

	delta_t = 0.04

	N = 2
	v_i_prev = np.zeros((3, N))

	xyt_swarm = np.zeros((3,1))

	v_i, xyt_i, v, xyt_swarm_next = safe_motion_controller(v_desired, theta_scale, p_i_mat, theta_vec,
	v_max, a_max, delta_t, N, v_i_prev, xyt_swarm)
	print(v_i)
	print(xyt_i)
	print(v)
	print(xyt_swarm_next)


if __name__ == '__main__':
	test_safe_motion_controller()