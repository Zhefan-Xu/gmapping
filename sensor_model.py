import numpy as np 
import math
from scipy.stats import norm 
from IntegrateScan import wraptopi
from GridMap import GridMap


# initial pose estimate 

# scan matching

def ray_casting(pose, gridmap, max_range):

    #produces estimates of laser measurements based on an inputted position 

    #pose: obtained from motion
    #gridmap: unique map for each particle
    
    #Questions: 
    #do we have to flip coordinates of the grid_map? 

    object_detect = gridmap.p_occ #0.2 #tunable parameter

    N = 360
    beam_res = 1. * np.pi/180 

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    grid_size = gridmap.resolution
    width_grid = gridmap.width
    length_grid = gridmap.length

    theta = wraptopi(theta)
    meas_range = 2*np.pi

    #determine where the LIDAR is wrt to the center of the robot 

    posX = math.floor(x/grid_size)
    posY = math.floor(y/grid_size)

    mapX = math.floor(posX)
    mapY = math.floor(posY)

    # global raycasting direction array
    start_direction =  theta # determine if this needs to be adjusted
    direction_array = np.array([start_direction + i*meas_range/N for i in range(N)]).reshape((1, N)) # 0 - 359

    #y_temp = np.array([[direction_array], [np.inf * np.ones(1, N)]])
    y_temp = np.concatenate((direction_array, float('inf') * np.ones((1, N))))

    # narrow down the raycasting range to a square shape whose center is current pose
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
            # if find a cell occupied
            if gridmap.p_map[i, j] >= gridmap.p_occ:
                # center of mass of cell
                xc = (i + 0.5) * gridmap.resolution
                yc = (j + 0.5) * gridmap.resolution

                # angle from cell to robot in global frame
                theta_scan_global = np.arctan2(yc - y, xc - x)
                # angle from cell to robot in robot frame
                #theta_scan_robot = theta_scan_global - theta

                # calculate distance
                r = np.sqrt((yc - y)**2 + (xc - x)**2)

                # find idx of beam closest to cell
                idx = np.argmin(np.absolute(wraptopi(theta_scan_global - direction_array)))

                # calculate how many scans go through same cell
                rng = np.floor(np.arctan(0.5*gridmap.resolution/r) / beam_res)

                for k in range(int(idx - rng), int(idx + rng + 1)):
                    if k < 0:
                        k = N + k
                    elif k > N - 1:
                        k = k - N
                    # if a smaller distance is found, use the smaller one
                    if r < y_temp[1, k]:
                        y_temp[1, k] = r

    y_est = y_temp
    return y_est

    '''
    deg_step = 1

    for k in range(0,360,deg_step):

        theta_ray = theta - ((90-k) * math.pi/180) #confirm

        dirX = math.cos(theta_ray)
        dirY = math.sin(theta_ray)

        deltaDistX = abs(1 / dirX)
        deltaDistY = abs(1 / dirY)

        hitwall = 0
        side = 2

        if(gridmap[mapY][mapX] == -1): 

            y_temp[1,k] = 0


        # print(" here curr pos ",posX," y ",posY," theta ",theta," mapx ",mapX," mapy ",mapY," map reads ",gridmap[mapX][mapY])
        # return [mapX,mapY]

        if (dirX < 0):
            stepX = -1
            sideDistX = (posX - mapX) * deltaDistX
    
        else:      
            stepX = 1
            sideDistX = (mapX + 1.0 - posX) * deltaDistX
    
        if (dirY < 0):      
            stepY = -1
            sideDistY = (posY - mapY) * deltaDistY
    
        else:      
            stepY = 1
            sideDistY = (mapY + 1.0 - posY) * deltaDistY

        while (hitWall == 0):
    
        
            if (sideDistX < sideDistY):
            
                sideDistX += deltaDistX
                mapX += stepX
                side = 0
                # print("in x dir delta ",deltaDistX," sideDistX ",sideDistX," mapx ",mapX," mapy ",mapY)
    
            else:
        
                sideDistY += deltaDistY
                mapY += stepY
                side = 1
                # print("in y dir delta ",deltaDistY," sideDistY ",sideDistY," mapx ",mapX," mapy ",mapY)

            if (gridmap[mapY][mapX] >= object_detect or gridmap[mapY][mapX] == -1):
                # print("hit wall at x ",mapX, " y ",mapY)
                hitWall = 1
        
        if(side == 0):
            perpWallDist = ((mapX - posX + (1 - stepX) / 2) / dirX)      
        else:        
            perpWallDist = (mapY - posY + (1 - stepY) / 2) / dirY
            
        y_temp[1,k] = perpWallDist

    y_est = y_temp
    '''
     



def calculate_P (y_laser, y_pred, sigma):

    # y_laser: 2D array of laser measurement results
    # y_pred:  2D array of laser predicted measurment results based on position
    # sigma: standard deviation 

    q = 1

    for i in range(y_laser.shape[1]):
        
        p_x = math.exp((-(y_laser[0,i] -y_pred[0,i])**2)/(2*sigma**2)) /np.sqrt(2*np.pi*sigma**2)
        p_y = math.exp((-(y_laser[1,i] -y_pred[1,i])**2)/(2*sigma**2)) /np.sqrt(2*np.pi*sigma**2)

        p = p_x*p_y

        q = q*p
        
    return q




