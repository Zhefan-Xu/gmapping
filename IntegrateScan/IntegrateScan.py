import numpy as np

def find_end(pose, direction, zi):
        
        map_resolution = 0.05
        res = 1

        step_size =  map_resolution * res
        x, y = pose[0], pose[1]
        end_x = x + zi * np.cos(direction)
        end_y = y + zi * np.sin(direction)

        end_x_index = int(end_x/map_resolution)
        end_y_index = int(end_y/map_resolution)

        return end_x_index, end_y_index

def integrateScan(prev_map, pose, z):
    # z: measurements, 1 by 360
    # assume z are all numbers
    N = z.shape[0]
    x = pose[0]
    y = pose[1]
    theta = pose[2]
    new_map = prev_map

    meas_range = 2*np.pi
    start_direction = theta - np.pi/2
    direction_array = np.array([start_direction + i*meas_range/N for i in range(N)]) # 0 - 359

    for i in range(N):
        index_x, index_y = find_end(pose, direction_array[i], z[i])
        new_map[index_x, index_y] = 1.0


    return new_map

if __name__ == "__main__":
    print('www')
    pass