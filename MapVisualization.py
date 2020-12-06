import numpy as np
from matplotlib import pyplot as plt


class MapVisualizer:
    def __init__(self,size_meters,pixels):
        # size_meters: estimated map size, e.g. 10m x 10m
        # pixels: map matrix size, e.g. [1000,1000]
        self.size_meters=size_meters
        self.pixels=pixels
        self.resolution=size_meters/pixels
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
        # self.ax.set_xlim([0, self.pixels])
        self.ax.set_ylim([0, self.pixels])
        ticks=np.arange(0,self.size_meters+1)
        labels = [str(tick) for tick in ticks]
        self.ax.set_xticks(ticks/self.resolution)
        self.ax.set_yticks(ticks/self.resolution)
        self.ax.set_xticklabels(labels)
        self.ax.set_yticklabels(labels)

    def visualize(self,X,map_matrix):
        # X: [x,y,theta]
        # map_matrix: same as hw1

        # If pre-processing on input values is needed:(from hw1)
        # map_matrix[map_matrix < 0] = -1
        # map_matrix[map_matrix > 0] = 1 - map_matrix[map_matrix > 0]

        if self.vehicle:
            self.vehicle.remove()
        self.vehicle=self.ax.arrow(X[0]/self.resolution, X[1]/self.resolution,
                0.1*np.cos(X[2]), 0.1*np.sin(X[2]), head_width=2, fc='r', ec='r')
        if self.img is None:
            self.img = self.ax.imshow(map_matrix.T, cmap='Greys')
        else:
            self.img.set_data(map_matrix.T)
        plt.pause(0.0001)
        plt.draw()



# # Example in 25 steps:
# map_visualizer=MapVisualizer(8,1000)
# for i in range(25):
#     X = [0.2*i, 0.3*i, i*np.pi / 4]
#     test=np.random.rand(1000000).reshape((1000,1000))
#     map_visualizer.visualize(X,test)


