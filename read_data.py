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

max_range = 3.5 # INF

def main():
	src_path_log = "./turtlebot_log.txt"
	logfile = open(src_path_log, 'r')
	print("=================GMAPPING==================")

	# Initialize Map and Visualizer
	length = 10
	width = 10
	map_resolution = 0.05
	gridmap = GridMap(map_resolution, length, width)
	print(gridmap)
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
	print(particles)



	for time_idx, line in enumerate(logfile):
		meas_type = line[0]
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
		if (time_idx == 0):
			u_t0 = np.fromstring(line[2:], dtype=np.float64, sep=' ')
			


		if (first_measurment and meas_type == "L"):		
			print(gridmap)
			meas_vals = meas_vals[:-1]
			gridmap.l_map, gridmap.p_map = integrateScan(gridmap, initial_pose, meas_vals, max_range)
			for i in range(num_particle):
				particles[i][1] = copy.deepcopy(gridmap)
			map_visualizer.visualize(initial_pose, gridmap.p_map)
			# plt.show(block=True)
			first_measurment = False

			meas_est = ray_casting(initial_pose, gridmap, max_range)
			print(meas_est[1,:])
		
		if (not first_measurment):
			for particle in particles:
				[pose, gridmap, weight] = particle

				# Odometry Measurement
				if meas_type == "O":
					u_t1 = meas_vals
					odom = u_t1[0:3] - u_t0[0:3]
					pose_motion_model = motion_model(pose, odom) # Add noise?
					particle[0] = pose_motion_model
					
				# Sensor Measurement
				elif meas_type == "L":
					# Step 1: get estimated sensor measurement from estimated pose and previous map
					meas_vals = meas_vals[:-1]
					meas_true = meas_vals # shape [1, 360] 
					#meas_est = ray_casting(pose, gridmap)

					# Step 2: Find the useable laser beams: range < max_range for both estimate and true
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

					# Step 3: convert measurement to pointcloud in world frame
					pc_true = meas_to_pointcloud(meas_true_usable, pose)
					pc_est = meas_to_pointcloud(meas_est_usable, pose)


					# Step 4: Perform ICP to get the tranform matrix
					tranform = ICP(pc_true, pc_est)

					# Step 5: correct pose based on trasform (If ICP succeed!!!!!)
					pose_corrected = transform * pose



			u_t0 = u_t1





if __name__ == "__main__":
	main()