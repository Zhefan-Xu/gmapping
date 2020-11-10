import numpy as np
from matplotlib import pyplot as plt


class MapVisualizer:
    def __init__(self,size_meters,resolution):
        # size_meters: estimated map size,e.g. [10,11] for  10m x 11m
        # resolution: meters/matrix_size, if the map=[10,11] and the matrix.shape=[1100,1000], resolution=0.01
        self.size_meters=size_meters
        self.pixels=size_meters/resolution
        self.resolution=resolution
        self.vehicle=None
        self.img=None
        self.setlabels()

    def setlabels(self):
        fig=plt.figure(figsize=(7,7))
        mng = plt.get_current_fig_manager()
        plt.ion()
        self.ax = fig.gca()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)
        plt.title('GMAPPING')
        self.ax.set_xlim([0, self.pixels[0]])
        self.ax.set_ylim([0, self.pixels[1]])
        xticks=np.arange(0,self.size_meters[0]+1)
        yticks=np.arange(0,self.size_meters[1]+1)
        xlabels = [str(xtick) for xtick in xticks]
        ylabels = [str(ytick) for ytick in yticks]
        self.ax.set_xticks(xticks/self.resolution)
        self.ax.set_yticks(yticks/self.resolution)
        self.ax.set_xticklabels(xlabels)
        self.ax.set_yticklabels(ylabels)

    def visualize(self,X,map_matrix):
        # X: [x,y,theta]
        # map_matrix: same as hw1

        # If pre-processing on input values is needed:(from hw1)
        # map_matrix[map_matrix < 0] = -1
        # map_matrix[map_matrix > 0] = 1 - map_matrix[map_matrix > 0]

        if self.vehicle:
            self.vehicle.remove()
        self.vehicle=self.ax.arrow(X[0]/self.resolution, X[1]/self.resolution,
                0.1*np.cos(X[2]), 0.1*np.sin(X[2]), head_width=10, fc='r', ec='r')
        if self.img is None:
            self.img = self.ax.imshow(map_matrix, cmap='Greys')
        else:
            self.img.set_data(map_matrix)
        plt.pause(0.0001)
        plt.draw()



# # Example in 25 steps:
# map=np.array([8,9])
# map_visualizer=MapVisualizer(map,0.01)
# for i in range(25):
#     X = [0.2*i, 0.3*i, i*np.pi / 4]
#     test=np.random.rand(720000).reshape((900,800))
#     map_visualizer.visualize(X,test)


