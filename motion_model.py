import numpy as np

def wrapToPi(theta):
	while (theta > np.pi):
		theta -= np.pi * 2

	while (theta < -np.pi):
		theta += np.pi * 2
	return theta


def motion_model(pose, odom):
	rot1 = wrapToPi(np.arctan2(odom[1], odom[0]) - pose[2])
	trans = np.sqrt(odom[0]**2 + odom[1]**2)
	rot2 = wrapToPi(odom[2] - rot1)

	x = pose[0] + trans * np.cos(pose[2] + rot1)
	y = pose[1] + trans * np.sin(pose[2] + rot1)
	theta = wrapToPi(pose[2] + rot1 + rot2)
	return np.array([x, y, theta])