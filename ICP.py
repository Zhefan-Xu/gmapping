import numpy as np
from sklearn.neighbors import NearestNeighbors

def cal_distance(p1, p2):
	diff_x = p1[0] - p2[0] 
	diff_y = p1[1] - p2[1]
	return np.sqrt(diff_x**2 + diff_y**2)


def find_closest_point(p, pc):
	min_distance = 100000
	closest_point = np.array([0, 0])
	closest_point_idx = -1
	for idx, point in enumerate(pc):
		distance = cal_distance(p, point)
		if (distance < min_distance):
			min_distance = distance
			closest_point = point
			closest_point_idx = idx

	return closest_point, closest_point_idx

def cal_error(pc_true, pc_est):
	total_distance = 0
	for point_est in pc_est:
		closest_point, closest_point_idx = find_closest_point(point_est, pc_true)
		distance = cal_distance(point_est, closest_point)
		total_distance += distance
	return total_distance



def ICP(pc_true, pc_est):
	"""
	pc_true: True measurement in global frame. shape: [360, 2]
	pc_est: Estimated measuremetn in global frame. Shape: [360, 2]
	"""

	error = 100000
	error_decrease = True
	error_thresh = 1e-3
	last_error = error + 100
	R_total = np.eye(2)
	T_total = np.zeros(2)
	# print(pc_true[:10], pc_est[:10])
	while (error > error_thresh or error_decrease):

		# Step 1: Calculate center and subtract cneter for each pointcloud data
		center_true = np.mean(pc_true, axis=0)
		center_est = np.mean(pc_est, axis=0)
		pc_true_norm = pc_true - center_true 
		pc_est_norm = pc_est - center_est

		# Step 2: Calculate the cross covariance matrix
		W = np.zeros((2,2)) # initialize the cross covirance matrix
		for point_est, point_est_norm in zip(pc_est, pc_est_norm):
			# Find the closest point:
			closest_point, closest_point_idx = find_closest_point(point_est, pc_true)

			# Incrementally add cross covariance matrix
			close_pc_true_norm = pc_true_norm[closest_point_idx]
			W += close_pc_true_norm @ point_est_norm.T

		# Step 3: Apply Numpy SVD decomposition: (Not sure)
		U, D, V_T = np.linalg.svd(W)

		# Step 4: Calculate the rotation matrix
		R = U @ V_T
		R_total = R @ R_total
		
		error = cal_error(pc_true, pc_est)
		print(error)


		# Step 5: Transform estimated point
		pc_est = (R @ pc_est_norm.T).T + center_true


		# Step 6: Evaluate the errors
		error = cal_error(pc_true, pc_est)
		print(error)
		error_decrease = (error - last_error) < 0
		print(error)
		last_error = error

	T = center_true - R_total @ center_est
	transform = np.zeros((3, 3))
	transform[0:2, 0:2] = R_total
	transform[0:2, 2] = T
	transform[2,2] = 1 

	return transform


