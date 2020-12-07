import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree as KDTree

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


def indexMean(indices,arrays):
    indexSum = np.array([0.0, 0.0])
    for i in range(np.size(indices,0)):
        indxSum = np.add(indexSum, np.array(arrays[indices[i]]), out = indexSum ,casting = 'unsafe')
    return indxSum/np.size(indices,0)

def indexTrue(indices, pc_true):
	pc_true_ = []
	for idx in indices:
		point = pc_true[idx]
		pc_true_.append(point)
	return np.array(pc_true_)


def ICP(pc_true, pc_est):
	"""
	pc_true: True measurement in global frame. shape: [360, 2]
	pc_est: Estimated measuremetn in global frame. Shape: [360, 2]
	"""

	ICP_failure = False

	# TEST PURPOSE
	angle = 30 * np.pi/180
	R_test = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
	center_est = np.mean(pc_est, axis=0)
	pc_est_norm = pc_est - center_est
	pc_est = (R_test @ pc_est_norm.T).T + center_est

	error = 100000
	error_decrease = True
	error_thresh = 1e-3
	error_list = [error]
	max_iteration = 100

	R_total = np.eye(2)
	T_total = np.zeros(2)
	kdtree = KDTree(pc_true)

	iteration = 0
	while (iteration < max_iteration and error > error_thresh):
		iteration += 1
		plt.figure(figsize=(7,7))
		plt.scatter(pc_true[:, 0], pc_true[:, 1], label="true")
		plt.scatter(pc_est[:,0], pc_est[:, 1], label="ests")
		
		plt.legend()
		plt.show()
		# print(iteration)

		distance, indices = kdtree.query(pc_est)
		error = np.mean(distance**2)
		# print(error)
		error_list.append(error)
		if (error > error_list[-2]):
			break
		
		# Calculate Center
		center_est = np.mean(pc_est, axis=0)
		center_true = indexMean(indices, pc_true)

		pc_true_ = indexTrue(indices, pc_true)

		pc_true_norm = pc_true_ - center_true
		pc_est_norm = pc_est - center_est
		

		# Cross Covaraince Matrix
		W = pc_true_norm.T @ pc_est_norm
		# print(W)

		U, D, V_T = np.linalg.svd(W)


		# Update Estimate:



		R = U @ V_T
		T = center_true - R @ center_est
		# print(R)
		# print(T)
		# print(pc_est[0])
		pc_est = (R @ pc_est.T).T + T

		# print(pc_est[0])
		# print(pc_true_[0])

		R_total = R @ R_total
		T_total = R @ T_total + T 

		# print(W)

		# print(pc_est[0])
		# print(center_est)
		# print(pc_est_norm[0])


		# print(pc_true[0])
		# print(center_true)
		# print(pc_true_norm[0])

		# print(pc_true.shape)
		# print(center_est)
		# print(center_true)

	transform = np.zeros((3, 3))
	transform[0:2, 0:2] = R_total
	transform[0:2, 2] = T_total
	transform[2,2] = 1

	# print(np.linalg.norm(T_total))

	if np.linalg.norm(T_total) > 0.001:
		ICP_failure = True 

	# plt.scatter(pc_true[:, 0], pc_true[:, 1], label="true")
	# plt.scatter(pc_est[:,0], pc_est[:, 1], label="ests")
	# plt.legend()
	# plt.show()


	return transform, ICP_failure


	# # print(pc_true[:10], pc_est[:10])
	# while (error > error_thresh or error_decrease):

	# 	# Step 1: Calculate center and subtract cneter for each pointcloud data
	# 	center_true = np.mean(pc_true, axis=0)
	# 	center_est = np.mean(pc_est, axis=0)
	# 	pc_true_norm = pc_true - center_true 
	# 	pc_est_norm = pc_est - center_est

	# 	# plt.scatter(pc_true[:, 0], pc_true[:, 1], label="true")
	# 	# plt.scatter(pc_est[:,0], pc_est[:, 1], label="ests")
	# 	# plt.legend()
	# 	# plt.show()
	# 	# return

	# 	# Step 2: Calculate the cross covariance matrix
	# 	W = np.zeros((2,2)) # initialize the cross covirance matrix
	# 	for point_est, point_est_norm in zip(pc_est, pc_est_norm):
	# 		# Find the closest point:
	# 		closest_point, closest_point_idx = find_closest_point(point_est, pc_true)

	# 		# Incrementally add cross covariance matrix
	# 		close_pc_true_norm = pc_true_norm[closest_point_idx]
	# 		W += close_pc_true_norm @ point_est_norm.T

	# 	# Step 3: Apply Numpy SVD decomposition: (Not sure)
	# 	U, D, V_T = np.linalg.svd(W)

	# 	# Step 4: Calculate the rotation matrix
	# 	R = U @ V_T
	# 	R_total = R @ R_total
		
	# 	error = cal_error(pc_true, pc_est)
	# 	print(error)


	# 	# Step 5: Transform estimated point
	# 	pc_est = (R @ pc_est_norm.T).T + center_true

	# 	# plt.scatter(pc_true[:, 0], pc_true[:, 1], label="true")
	# 	# plt.scatter(pc_est[:,0], pc_est[:, 1], label="ests")
	# 	# plt.legend()
	# 	# plt.show()
	# 	# return


	# 	# Step 6: Evaluate the errors
	# 	error = cal_error(pc_true, pc_est)
	# 	print(error)
	# 	error_decrease = (error - last_error) < 0
	# 	# print(error)
	# 	last_error = error

	# T = center_true - R_total @ center_est
	# transform = np.zeros((3, 3))
	# transform[0:2, 0:2] = R_total
	# transform[0:2, 2] = T
	# transform[2,2] = 1 

	# return transform


