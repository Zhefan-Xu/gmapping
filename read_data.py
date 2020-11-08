import numpy as np

max_range = 3.5 # INF

def main():
	src_path_log = "./turtlebot_log.txt"
	logfile = open(src_path_log, 'r')

	for time_idx, line in enumerate(logfile):
		meas_type = line[0]
		meas_vals = []
		
		meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')

		# Odometry Measurement
		if meas_vals == "O":
			pass
		# Sensor Measurement
		elif meas_vals == "L":
			pass





if __name__ == "__main__":
	main()