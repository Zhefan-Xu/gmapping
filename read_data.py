import numpy as np
import sys
import matplotlib.pyplot as plt
sys.path.append('./IntegrateScan')
from GridMap import GridMap
from IntegrateScan import integrateScan
from MapVisualization import MapVisualizer
from motion_model import *
from sensor_model import *
import copy

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
			gridmap.l_map, gridmap.p_map = integrateScan(gridmap, initial_pose, meas_vals, max_range)
			for i in range(num_particle):
				particles[i][1] = copy.deepcopy(gridmap)
			# np.savetxt("1stmap.txt", gridmap.p_map, fmt="%.2f")
			# plt.imshow(np.rot90(gridmap.p_map, 1, (1, 0)))
			map_visualizer.visualize(initial_pose, gridmap.p_map)
			# plt.imshow(gridmap.p_map)
			# plt.plot(initial_pose[0], initial_pose[1], 'x')
			#plt.colorbar()
			plt.show(block=True)
			first_measurment = False
		
		if (not first_measurment):
			for particle in particles:
				[pose, gridmap, weight] = particle

				# Odometry Measurement
				if meas_type == "O":
					u_t1 = meas_vals
					odom = u_t1[0:3] - u_t0[0:3]
					pose_motion_model = motion_model(pose, odom)
					particle[0] = pose_motion_model
					
				# Sensor Measurement
				elif meas_type == "L":
					# Step 1: get estimated sensor measurement from estimated pose and previous map
					meas_true = meas_vals 
					print(meas_true)
					return
					meas_est = ray_casting(pose, gridmap)





			u_t0 = u_t1





if __name__ == "__main__":
	main()