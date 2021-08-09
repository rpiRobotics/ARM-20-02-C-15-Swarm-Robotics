#!/usr/bin/env python
import numpy as np

class EKF:
	def __init__(self, uwb_meas_std, odom_meas_std, process_pos_std, process_vel_std):
		self.uwb_meas_cov = np.diag(uwb_meas_std)**2
		self.odom_meas_cov = np.diag(odom_meas_std)**2
		self.process_pos_std = process_pos_std
		self.process_vel_std = process_vel_std

	def EKF_UWB(self,state, cov, dt, meas, rmse):
		'''
		Kalman filter iteration using an UWB reading

		Inputs:
			state: Robot state from previous timestep
				[x; y; theta; x_d; y_d; theta_d]
			cov: State covariance matrix from previous timestep
			dt: Time (in seconds) since previous timestep
			meas: UWB measurement (from multilaterion)
				[x;y;theta]
			rmse: UWB multilateration RMSE
				(used to adjust measurement covariance)
		
		Outputs:
			state_c: State vector (after prediction/correction)
			cov_c: State covariance matrix (after prediction/correction)
			pos: Kalman estimate of robot position (first 3 entries of state_c)
		'''
		C_pos = np.array([
				[1., 0., 0., 0., 0., 0.],
				[0., 1., 0., 0., 0., 0.],
				[0., 0., 1., 0., 0., 0.]])

		def measurementfcn(x):
			return C_pos.dot(x)

		def dh_dx_fcn(x):
			return C_pos

		def dh_dw_fcn(x):
			return np.eye(3)

		# meas_cov = np.diag(rmse*np.array([1., 1., 1.]))**2 # TODO
		# meas_cov = np.diag(((rmse/0.1)*np.array([0.5, 0.5, 0.2]))**2) 
		# meas_cov = np.diag(np.array([0.5, 0.5, 0.2])**2) 

		meas_angle_ind = 2;
		state_angle_ind = 2;

		state_p, cov_p, meas_p = self.EKF_constant_vel_predict(
			state, cov, measurementfcn, meas_angle_ind, dt)

		state_c, cov_c, meas_c = self.EKF_correct(measurementfcn, dh_dx_fcn, dh_dw_fcn,
		state_p, cov_p, meas_p, meas, self.uwb_meas_cov,
		state_angle_ind, meas_angle_ind)

		pos = meas_c
		return state_c, cov_c, pos

	def EKF_odom(self,state, cov, dt, meas):
		'''
		Kalman filter iteration using an odometry reading

		Inputs:
			state: Robot state from previous timestep
				[x; y; theta; x_d; y_d; theta_d]
			cov: State covariance matrix from previous timestep
			dt: Time (in seconds) since previous timestep
			meas: odometry measurement
				[x_d;y_d;theta_d] in the robot frame
		
		Outputs:
			state_c: State vector (after prediction/correction)
			cov_c: State covariance matrix (after prediction/correction)
			pos: Kalman estimate of robot position (first 3 entries of state_c)
		'''

		def measurementfcn(x):
			xy = rot_z_2d(-x[2][0]).dot(x[3:4+1,:])
			return np.block([[xy], [x[5]]])

		def dh_dx_fcn(x):
			c = np.cos(x[2][0])
			s = np.sin(x[2][0])

			J_13 =  x[4][0]*c-x[3][0]*s
			J_23 = -x[3][0]*c-x[4][0]*s

			return np.array([
				[0., 0., J_13,  c,  s,  0.],
				[0., 0., J_23, -s,  c,  0.],
				[0., 0., 0.,    0., 0., 1.]])

		def dh_dw_fcn(x):
			return np.eye(3)

		# meas_cov = np.diag(np.array([0.0001, 0.0001, 0.00005])**2)

		meas_angle_ind = [];
		state_angle_ind = 2;


		state_p, cov_p, meas_p = self.EKF_constant_vel_predict(
			state, cov, measurementfcn, meas_angle_ind, dt)

		state_c, cov_c, meas_c = self.EKF_correct(measurementfcn, dh_dx_fcn,dh_dw_fcn,
		state_p, cov_p, meas_p, meas, self.odom_meas_cov,
		state_angle_ind, meas_angle_ind)

		pos_c = state_c[0:2+1, :]
		return state_c, cov_c, pos_c

	def EKF_constant_vel_predict(self,state, cov, measurementfcn, meas_angle_ind, dt):
		'''
		Kalman filter prediction with a constant velocity model

		Inputs:
			measurementfcn(state): Measurement function
			meas_angle_ind: Indices of measurement variables which are angles
				(will be wrapped)
			dt: Timestep to take when making the prediction
		Outputs:
			state_p: Predicted state variable after dt
			cov_p: Predicted state covariance matrix after dt
			meas_p: Predicted measurement after dt
		'''

		def A(T):
			return np.array([
				[1., 0., 0.,  T, 0., 0.],
				[0., 1., 0., 0.,  T, 0.],
				[0., 0., 1., 0., 0.,  T],
				[0., 0., 0., 1., 0., 0.],
				[0., 0., 0., 0., 1., 0.],
				[0., 0., 0., 0., 0., 1.]])
		
		def transitionfcn(x,T):
			return A(T).dot(x)
		
		def df_dx_fcn(x,T):
			return A(T)

		def df_dw_fcn(x):
			return np.eye(6)

		# process_cov = dt**2 * np.diag([0.3, 0.3, 0.4, 0., 0., 0.])**2 \
		# 			+ dt * np.diag([0., 0., 0., 0.3, 0.3, 0.4])**2

		process_cov = dt**2 * np.diag(self.process_pos_std + [0., 0., 0.])**2 \
					+ dt * np.diag([0., 0., 0.] + self.process_vel_std)**2

		state_angle_ind = 2;

		return self.EKF_predict(transitionfcn, measurementfcn,
		df_dx_fcn, df_dw_fcn, process_cov, state, cov, dt,
		state_angle_ind, meas_angle_ind)


	def EKF_predict(self,transitionfcn, measurementfcn,
		df_dx_fcn, df_dw_fcn, process_cov, state, cov, dt,
		state_angle_ind, meas_angle_ind):
		'''
		Extended Kalman filter prediction step
		
		Inputs:
			transitionfcn(state, dt): State transition function
			measurementfcn(state): Measurement function
		
			df_dx_fcn(state, dt): Function returning d(transitionfcn)/d(state)
			df_dw_fcn(state): Function returning d(transitionfcn)/d(noise)
			process_cov: Process noise covariance matrix
			state: State vector at previous timestep
			cov: State covariance matrix at previous timestep
			dt: Timestep to take when making the prediction
		
			state_angle_ind: Indices of state variables which are angles
				(will be wrapped)
			meas_angle_ind: Indices of measurement variables which are angles
				(will be wrapped)

		Outputs:
			state_p: Predicted state variable after dt
			cov_p: Predicted state covariance matrix after dt
			meas_p: Predicted measurement after dt
		'''
		state_p = transitionfcn(state, dt)
		state_p[state_angle_ind] = wrapToPi(state_p[state_angle_ind])

		df_dx = df_dx_fcn(state, dt)
		df_dw = df_dw_fcn(state)
		cov_p = df_dx.dot(cov).dot(df_dx.T) + df_dw.dot(process_cov).dot(df_dw.T)

		meas_p = measurementfcn(state_p)
		meas_p[meas_angle_ind] = wrapToPi(meas_p[meas_angle_ind])

		cov_p = 0.5 * (cov_p + cov_p.T) # to make sure the covariance matrix is symmetric 

		return state_p, cov_p, meas_p

	def EKF_correct(self,measurementfcn, dh_dx_fcn,dh_dw_fcn,
		state_p, cov_p, meas_p, meas, meas_cov,
		state_angle_ind, meas_angle_ind):
		'''
		Extended Kalman filter correction step

		Inputs:
			measurementfcn(state): Measurement function
			dh_dx_fcn(state): Function returning d(measurementfcn)/d(state)
			dh_dw_fcn(state): Function returning d(measurementfcn)/d(noise)

			state_p: Predicted state variable (from prediction step)
			cov_p: Predicted state covariance matrix (from prediction step)
			meas_p: Predicted measurement (from prediction step)

			meas: Actual measurement
			meas_cov: Covariance matrix of measurement

			state_angle_ind: Indices of state variables which are angles
				(will be wrapped)
			meas_angle_ind: Indices of measurement variables which are angles
				(will be wrapped)
		Outputs:
			state_p: Corrected state variable
			cov_c: Corrected state covariance matrix
			meas_c: Corrected measurement
		'''
		dh_dx = dh_dx_fcn(state_p)
		dh_dw = dh_dw_fcn(state_p)
		S = dh_dx.dot(cov_p).dot(dh_dx.T) + dh_dw.dot(meas_cov).dot(dh_dw.T)
		#K = cov_p.dot(dh_dx.T).dot(np.linalg.inv(S+1.0e-6*np.eye(6)))
		K = cov_p.dot(dh_dx.T).dot(np.linalg.inv(S))
		# TODO don't use inverse directly ^^^

		meas_diff = meas - meas_p
		meas_diff[meas_angle_ind] = wrapToPi(meas_diff[meas_angle_ind]);

		state_c = state_p + K.dot(meas_diff);
		state_c[state_angle_ind] = wrapToPi(state_c[state_angle_ind]);

		cov_c = cov_p - K.dot(S).dot(K.T)
		meas_c = measurementfcn(state_c);
		meas_c[meas_angle_ind] = wrapToPi(meas_c[meas_angle_ind]);

		cov_c = 0.5 * (cov_c + cov_c.T)

		return state_c, cov_c, meas_c

def wrapToPi(a):
	'''
	Wraps angle to [-pi,pi)
	'''
	return ((a+np.pi) % (2*np.pi))-np.pi

def rot_z_2d(theta):
	'''
	2D rotation matrix
	'''
	c, s = np.cos(theta), np.sin(theta)
	return np.array([[c, -s], [s, c]])

def test_EKF():
	uwb_meas_std = [0.5, 0.5, 0.2]
	odom_meas_std = [0.0001, 0.0001, 0.00005]
	process_pos_std = [0.3, 0.3, 0.4]
	process_vel_std = [0.3, 0.3, 0.4]

	ekf = EKF(uwb_meas_std, odom_meas_std, process_pos_std, process_vel_std)

	state = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
	cov = 100.0**2*np.eye(6)
	dt = 0.1
	UWB_meas = np.array([[1.],[1.],[0.5]])
	rmse = 0.1
	state_c, cov_c, pos_c = ekf.EKF_UWB(state, cov, dt, UWB_meas, rmse)
	print(state_c)
	print(cov_c)
	print(pos_c)


	print('----------')
	odom_meas = np.array([[0.],[0.],[0.]])
	state_c, cov_c, pos_c = ekf.EKF_odom(state_c, cov_c, dt, odom_meas)
	print(state_c)
	print(cov_c)
	print(pos_c)

if __name__ == '__main__':
	test_EKF()