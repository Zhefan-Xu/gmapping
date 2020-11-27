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


if __name__ == "__main__":
	# test SVD
	W = np.array([[1, 2],[3, 4]])
	WT = W.T
	U1, _, VT1 = np.linalg.svd(W)
	U2, _, VT2 = np.linalg.svd(WT)
	print(U1.dot(VT1))
	print(VT2.T.dot(U2.T))
	print(U1,'\n', VT1)
	print(U2,'\n', VT2)