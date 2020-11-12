import numpy as np
import sys
sys.path.append('./IntegrateScan')
from GridMap import GridMap
from IntegrateScan import integrateScan
from MapVisualization import MapVisualizer

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
	X = [0, 0, 0]


	for time_idx, line in enumerate(logfile):
		meas_type = line[0]
		
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')

		# Odometry Measurement
		if meas_type == "O":
			pass
		# Sensor Measurement
		elif meas_type == "L":
			if first_measurment:
				gridmap.l_map, gridmap.p_map = integrateScan(gridmap, X, meas_vals, max_range)
				print(gridmap.p_map)
			pass





if __name__ == "__main__":
	main()