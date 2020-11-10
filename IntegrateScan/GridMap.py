import numpy as np
import math

class GridMap:

    
    def __init__(self, map_resolution, length, width):
        size_x = int(length/map_resolution)
        size_y = int(width/map_resolution)

        #self.grid_map = np.zeros((size_x, size_y))
        self.length = length
        self.width = width
        self.resolution = map_resolution
        self.p_map = 0.5 * np.ones((size_x, size_y)) # normalized p
        self.l_map = np.zeros((size_x, size_y))  # log odds
        self.p0 = 0.5
        self.p_occ = 0.9
        self.p_free = 0.3
        self.l0 = self.log_odds_ratio(self.p0)
        self.l_occ = self.log_odds_ratio(self.p_occ)
        self.l_free = self.log_odds_ratio(self.p_free)
        
    
    def log_odds_ratio(self, p):
        return np.log(p/(1-p))


if __name__ == "__main__":
    pass