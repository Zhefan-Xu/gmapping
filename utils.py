import numpy as np

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


def sample_around(pose, delta, K, sigma_x, sigma_y, sigma_theta):
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
			dtheta = sigma_th * np.random.randn()
			exit = abs(dtheta) < delta[2]
		theta_temp = theta + dtheta

		pose_samples[:, i] = np.array([x_temp, y_temp, theta_temp]).T


if __name__ == "__main__":
	print(np.array([1,2,3]).shape)