import numpy as np
import sys
import matplotlib.pyplot as plt
sys.path.append('./IntegrateScan')
from GridMap import GridMap
from IntegrateScan import integrateScan
from MapVisualization import MapVisualizer
from motion_model import *
from sensor_model import *
from utils import *
from ICP import ICP
import copy

from tqdm import tqdm

max_range = 3.5 # INF

def main():
	src_path_log = "./turtlebot_log.txt"
	logfile = open(src_path_log, 'r')
	print("=================GMAPPING==================")

	# Initialize Map and Visualizer
	length = 10
	width = 10
	map_resolution = 0.1
	gridmap = GridMap(map_resolution, length, width)

	pixels = gridmap.size_x
	map_visualizer = MapVisualizer(length, pixels)

	# First Measurment:
	first_measurment = True
	num_sample_particle = 10
	num_particle = 3

	initial_pose = np.array([4, 6, 0]) # Initialization for pose
	#X = np.array([0, 0, 0]) # Initialization for pose

	# initialzie particle
	particles = []
	for i in range(num_particle):
		particle = [0] * 3 # x, y, theta
		particle[0] = initial_pose
		weight = 1/num_particle
		particle[2] = weight
		particles.append(particle)
	particles = np.array(particles)

	count_meas = 0
	for time_idx, line in enumerate(logfile):
		print(time_idx)
		meas_type = line[0]
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')


		if (meas_type == "O"):
			meas_odom = meas_vals[:-1]



		# Only perform computation when new scan receives
		if (meas_type == "L"):
			# First time: Initialze map for each particle
			if (first_measurment):
				meas_scan = meas_vals[:-1] # Last measurement is time
				gridmap.l_map, gridmap.p_map = integrateScan(gridmap, initial_pose, meas_scan, max_range)
				for i in range(num_particle):
					particles[i][1] = copy.deepcopy(gridmap)
				map_visualizer.visualize(initial_pose, gridmap.p_map)
				u_t0 = meas_odom
				first_measurment = False

				print("Initialze")
				print(particles)

			else:
				count_meas += 1
				print("Laser ID", count_meas)
				print(particles)

				# Get the lastest odometry measurement
				u_t1 = meas_odom
				odom = u_t1 - u_t0
				meas_scan = meas_vals[:-1]

				# Plot particle with the largest weights:
				particles_weights = particles[:, 2]
				max_weight_idx = np.argmax(particles_weights)
				max_particle_pose = particles[max_weight_idx, 0]
				max_particle_map = particles[max_weight_idx, 1]
				map_visualizer.visualize(max_particle_pose, max_particle_map.p_map)

				# Iterate through every particle:
				for p_idx, particle in enumerate(particles):
					[pose, gridmap, weight] = particle
					
					# map_visualizer.visualize(initial_pose, gridmap.p_map)
					# Step 1: Scan Matching:
					## Update Odometry
					pose_motion_model = motion_model(pose, odom)
					## ICP Scan Matcher
					"""
					1. Estimate measurment from previous map with new pose
					2. Find usable measurment from both true measurment and estimate measuremnt
					3. Perform ICP to find tranformation matrix
					"""

					# SM1: Find estiamted measurement
					meas_true = meas_scan
					meas_est = ray_casting(pose_motion_model, gridmap, max_range)[1,:] # Estimate Meas
					
					# SM2: Find the useable laser beams: range < max_range for both estimate and true
					meas_true_usable = [] 
					meas_est_usable = []
					for idx in range(len(meas_true)):
						range_true = meas_true[idx]
						range_est = meas_est[idx]
						if (range_true < max_range and range_est < max_range):
							bearing = idx * np.pi/180
							current_meas_true = [bearing, range_true]
							current_meas_est = [bearing, range_est]
							meas_true_usable.append(current_meas_true)
							meas_est_usable.append(current_meas_est)
					meas_true_usable = np.array(meas_true_usable)
					meas_est_usable = np.array(meas_est_usable)

					# SM3: convert measurement to pointcloud in world frame
					pc_true = meas_to_pointcloud(meas_true_usable, pose_motion_model)
					pc_est = meas_to_pointcloud(meas_est_usable, pose_motion_model)

					# SM4: Perform ICP to get the tranform matrix
					transform, ICP_failure = ICP(pc_true, pc_est)

					# Update pose based on scan matching results:
					R = transform[0:2,0:2]
					T = transform[0:2, 2]
					rotation_angle = np.arcsin(R[1,0])
					corrected_position = R @ pose_motion_model[0:2] + T 
					corrected_theta = wrapToPi(rotation_angle + pose[2])
					corrected_pose = np.array([corrected_position[0], corrected_position[1], corrected_theta])

					if (ICP_failure):
						# TODO 1: Sample new pose from Gaussian Motion Model using previous pose and current odom
						# Old pose is variable "pose", current odom is variable "odom"
						new_pose = sample_motion_model(pose, odom)
						# print(new_pose)

						meas_est = ray_casting(new_pose, gridmap, max_range)[1, :]
						# print(meas_est.shape)
						# print(meas_true.shape)
						# log probability from sensor model
						log_prob_SM = calculate_P(meas_true.reshape(1, -1), meas_est.reshape(1, -1))

						# TODO 2: Update particle weights based on Gaussin pdf w = w * p
						
						p = np.exp(log_prob_SM)
						# p_ = np.exp(log_prob_SM - logsumexp(log_prob_SM))
						# print(p)
						# print(p_)
						new_weight = particles[p_idx][2] * p


					else:
						print("ICP")
						# TODO 1: Sample k new pose around mode within delta range: 
						# For 1: K: 
							# Sample new pose
						pose_samples = sample_around(corrected_pose)


						# TODO 2: Compute Gaussian Mean and Vector for new pose:
						mu, cov, nf = compute_gaussian(pose_samples, gridmap, pose, meas_true, max_range, odom)
						# print(mu)
						# print(cov)

						# TODO 3: Draw sample from the derived Gaussian Distribution
						new_pose = np.random.multivariate_normal(mu, cov)
						# print(new_pose)


						# TODO 4: Update importance weights
						## w = w * NF
						new_weight = weight * nf

					# TODO: Update map with new pose and current measurment
					## m = integrateScan(...)
					gridmap.l_map, gridmap.p_map = integrateScan(gridmap, new_pose, meas_scan, max_range) 

					# TODO: Update Sample Set
					particles[p_idx][0] = new_pose
					particles[p_idx][1] = gridmap
					particles[p_idx][2] = new_weight

					# print("Update Samples")
					# print(particles)

				# Normalize 
				particles[:, 2] /= np.sum(particles[:, 2])

				# TODO: Resampling
				sqr_sum_weight = np.sum(particles[:, 2]**2)
				N_eff = 1/sqr_sum_weight


				resample_pecentage = 0.5
				threshold = resample_pecentage * num_particle # not sure
				weights_arr = particles[:, 2]
				if (N_eff < threshold):
					new_particles = []
					# print(weights_arr)
					weights_arr_ = []
					for w in weights_arr:
						weights_arr_.append(w)
					indices = np.random.choice(num_particle, num_particle, p=weights_arr_)
					for idx in indices:
						# print("index")
						# print(particles[idx])
						new_particles.append(particles[idx])

					particles = np.array(new_particles)
					particles[:, 2] /= np.sum(particles[:, 2])
				# Update u_t0
				u_t0 = u_t1
				



if __name__ == "__main__":
	main()
	plt.show()