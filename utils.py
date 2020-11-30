import numpy as np
from sensor_model import *
from motion_model import *
from scipy.special import logsumexp

def meas_to_pointcloud(meas, pose):
	[x, y, theta] = pose
	pointcloud = []
	for measurement in meas:
		[bearing, beam_range] = measurement
		pc_x = x + np.cos(bearing + theta) * beam_range
		pc_y = y + np.sin(bearing + theta) * beam_range
		pc = [pc_x, pc_y]
		pointcloud.append(pc)
	pointcloud = np.array(pointcloud)
	return pointcloud


def sample_around(pose, delta=[0.02, 0.02, 1e-3], K=10, sigma_x=0.05, sigma_y=0.05, sigma_theta=5e-4):
	'''
	pose: 3 by 1
	delta: 3 by 1
	'''
	x = pose[0]
	y = pose[1]
	theta = pose[2]

	pose_samples = np.zeros((3, K))
	for i in range(K):
		exit = False
		while not exit:
			dx = sigma_x * np.random.randn()
			exit = abs(dx) < delta[0]
		x_temp = x + dx
		exit = False
		while not exit:
			dy = sigma_y * np.random.randn()
			exit = abs(dy) < delta[1]
		y_temp = y + dy
		exit = False
		while not exit:
			dtheta = sigma_theta * np.random.randn()
			exit = abs(dtheta) < delta[2]
		theta_temp = theta + dtheta
		pose_samples[:, i] = np.array([x_temp, y_temp, theta_temp]).T
	return pose_samples

def compute_gaussian(pose_samples, gridmap, pose, meas_true, max_range, odom, a, sig_s):
	'''
	pose_samples: samples array of current particle
	gridmap: gridmap object of current particle
	pose: old pose
	meas_true: current scan meas
	max_range: sensor max_range
	odom: odometery
	a: alphas for motion model
	sig_s: sigma for semsor model
	
	return: mean and covariance matrix of gaussian

	need to use sensor model and motion model in this function
	'''

	mu = np.zeros((3,1))
	yita = 0
	cov = np.zeros((3, 3))

	k = pose_samples.shape[1]
	prob = np.zeros(k,1)

	for i in range(k):
		sample_i = pose_samples[:, i]

		# log probability from motion mmodel
		log_prob_MM = motion_model_odometry(sample_i, odom, pose, a)

		# ray casting based on sample_i
		meas_est = ray_casting(sample_i, gridmap, max_range)[1, :]
		# log probability from sensor model
		log_prob_SM = calculate_P(meas_true, meas_est, sig_s)

		# probability 
		prob[i, :] = np.exp((log_prob_MM + log_prob_SM) - logsumexp(log_prob_MM + log_prob_SM))

		mu = mu + sample_i * prob[i, :]
		yita = yita + prob[i, :]

	mu = mu / yita

	for i in range(k):
		sample_i = pose_samples[:, i]
		cov = cov + (sample_i - mu).dot((sample_i - mu).T) * prob[i, :]

	cov = cov / yita


	


	return mu, cov, yita

if __name__ == "__main__":
	print(np.array([1,2,3]).shape)