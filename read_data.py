import numpy as np
import sys
import matplotlib.pyplot as plt
sys.path.append('./IntegrateScan')
from GridMap import GridMap
from IntegrateScan import integrateScan
from MapVisualization import MapVisualizer
from motion_model import *
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
	pixels = gridmap.size_x
	map_visualizer = MapVisualizer(length, pixels)

	# First Measurment:
	first_measurment = True
	num_sample_particle = 10
	num_particle = 3

	X = np.array([5, 3, np.pi/2]) # Initialization for pose
	#X = np.array([0, 0, 0]) # Initialization for pose

	# initialzie particle
	particles = []
	for i in range(num_particle):
		particle = [0] * 3 # x, y, theta
		particle[0] = X
		weight = 1/num_particle
		particle[2] = weight
		particles.append(particle)
	particles = np.array(particles)



	plot_poses = []
	for time_idx, line in enumerate(logfile):
		meas_type = line[0]
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
		if (time_idx == 0):
			u_t0 = np.fromstring(line[2:], dtype=np.float64, sep=' ')
			


		if (first_measurment):
			if meas_type == "O":
				pass
			elif meas_type == "L":
				integrate_X = X
				#integrate_X[0], integrate_X[1] = X[1], X[0]
				print(meas_vals)
				gridmap.l_map, gridmap.p_map = integrateScan(gridmap, integrate_X, meas_vals, max_range)
				for i in range(num_particle):
					particles[i][1] = copy.deepcopy(gridmap)
				# np.savetxt("1stmap.txt", gridmap.p_map, fmt="%.2f")
				plt.imshow(np.rot90(gridmap.p_map, 1, (1, 0)))
				#plt.colorbar()
				plt.show(block=True)
				first_measurment = False
		else:
			for particle in particles:
				[pose, gridmap, weight] = particle

				# Odometry Measurement
				if meas_type == "O":
					u_t1 = meas_vals
					odom = u_t1[0:3] - u_t0[0:3]
					pose_motion_model = motion_model(pose, odom)
					# print(odom)
					# print(pose_motion_model)
					particle[0] = pose_motion_model
					plot_poses.append(pose_motion_model)
					

				# Sensor Measurement
				elif meas_type == "L":
					pass
			u_t0 = u_t1


	plot_poses = np.array(plot_poses)
	print(plot_poses)
	plt.plot(plot_poses[:,0], plot_poses[:, 1])
	plt.xlim(0,10)
	plt.ylim(0,10)
	plt.show()


if __name__ == "__main__":
	main()