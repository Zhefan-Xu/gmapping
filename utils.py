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


	