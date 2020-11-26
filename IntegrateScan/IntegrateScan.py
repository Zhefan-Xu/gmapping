import numpy as np
from GridMap import GridMap

def wraptopi(theta):

    if np.ndim(theta) == 0:

        while (theta > np.pi):
            theta -= np.pi * 2 

        while (theta < -np.pi):
            theta += np.pi * 2
    else:
        
        theta[theta > np.pi] -= np.pi * 2
        theta[theta < -np.pi] += np.pi * 2

    return theta

def find_end(pose, direction, zi):
        
        map_resolution = 0.1
        res = 1

        step_size =  map_resolution * res
        x, y = pose[0], pose[1]
        end_x = x + zi * np.cos(direction)
        end_y = y + zi * np.sin(direction)

        end_x_index = int(end_x/map_resolution)
        end_y_index = int(end_y/map_resolution)

        return end_x_index, end_y_index

def inverse_range_sensor_model(ii, jj, gridmap, pose, z, direction_array, max_range):
    '''
    Lecture occupancy grids slide 41
    gridmap is an object of GridMap
    '''
    
    alpha = 0.2   # thickness of obstacles
    beta = 0.02   # width of sensor beam

    # center of mass of cell
    xc = (ii + 0.5) * gridmap.resolution
    yc = (jj + 0.5) * gridmap.resolution

    # Calculate distance between psotion cell and studied cell
    r = np.sqrt((xc-pose[0])**2 + (yc-pose[1])**2)

    #Calculate angle between psotion cell and studied cell in global frame
    theta = np.arctan2(yc-pose[1],xc-pose[0])

    # find the closest beam index
    k = np.argmin(np.absolute(wraptopi(theta - direction_array)))

    # if the cell center is outside the beam range
    if r > min(max_range, z[k]) + alpha/2 or np.abs(wraptopi(theta - direction_array[k])) > beta/2:
        return gridmap.l0
    elif r < max_range and np.abs(r - z[k]) < alpha/2: # if it is occupied
        return gridmap.l_occ
    elif r <= z[k]:   # if it is free
        return gridmap.l_free


def integrateScan(gridmap, pose, z, max_range):
    # z: measurements, 1 by 360
    # assume z are all numbers
    N = z.shape[0]
    x = pose[0]
    y = pose[1]
    theta = pose[2]
    new_l_map = gridmap.l_map
    new_p_map = gridmap.p_map

    meas_range = 2*np.pi
    # start_direction = theta - np.pi/2
    start_direction = theta
    direction_array = np.array([start_direction + i*meas_range/N for i in range(N)]) # 0 - 359

    # narrow down the update range to a square shape whose center is current pose
    x_lb = max(0, x - max_range)
    x_ub = min(gridmap.length, x + max_range)
    y_lb = max(0, y - max_range)
    y_ub = min(gridmap.width, y + max_range)
    index_x_lb = np.floor(x_lb/gridmap.resolution).astype(int)
    index_x_ub = np.floor(x_ub/gridmap.resolution).astype(int)
    index_y_lb = np.floor(y_lb/gridmap.resolution).astype(int)
    index_y_ub = np.floor(y_ub/gridmap.resolution).astype(int)

    for i in range(index_x_lb, index_x_ub):
        for j in range(index_y_lb, index_y_ub):
            new_l_map[i,j] = gridmap.l_map[i,j] + inverse_range_sensor_model(i, j, gridmap, pose, z, direction_array, max_range) - gridmap.l0
    
    # Calculate normalized probablity form log-odds
    new_p_map = 1 - (1. / (1 + np.exp(new_l_map)))

    # return np.flipud(new_l_map), np.flipud(new_p_map)
    return new_l_map, new_p_map

if __name__ == "__main__":
    pose_theta = np.pi/3
    theta = np.pi
    meas_range = 2*np.pi
    start_direction = theta - np.pi/2
    N = 360
    direction_array = np.array([start_direction + i*meas_range/N for i in range(N)]) # 0 - 359
    k = np.argmin(np.absolute(theta - direction_array))
    print(k)
    pass