import numpy as np 
import math
from scipy.stats import norm 
import sys
from motion_model import wrapToPi
from GridMap import GridMap


# initial pose estimate 

# scan matching

def ray_casting(pose, gridmap, laser_res, max_range, grid_map, grid_area):

    #produces estimates of laser measurements based on an inputted position 

    #pose: obtained from motion
    #gridmap: unique map for each particle
    
    #Questions: 
    #do we have to flip coordinates of the grid_map? 

    object_detect = 0.2 #tunable parameter

    N = 360 

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    grid_size = gridmap.resolution
    width_grid = gridmap.width
    length_grid = gridmap.length

    theta = wrapToPi(theta)
    meas_range = 2*np.pi

    #determine where the LIDAR is wrt to the center of the robot 

    posX = math.floor(x/grid_size)
    posY = math.floor(y/grid_size)

    mapX = math.floor(posX)
    mapY = math.floor(posY)

    num_proj = floor(radius/laser_res)

    start_direction = 0 # determine if this needs to be adjusted

    y_temp = np.array([[start_direction + i*meas_range/N for i in range(N)],[np.ones(1, num_proj)]]) # 0 - 359

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

            if (gridmap[mapY][mapX] > object_detect or gridmap[mapY][mapX] == -1):
                # print("hit wall at x ",mapX, " y ",mapY)
                hitWall = 1
        
        if(side == 0):
            perpWallDist = ((mapX - posX + (1 - stepX) / 2) / dirX)      
        else:        
            perpWallDist = (mapY - posY + (1 - stepY) / 2) / dirY
            
        y_temp[1,k] = perpWallDist

    y_est = y_temp

    return y_est

        #accessing grid map 



def calculate_P(y_laser, y_pred, sigma):

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




