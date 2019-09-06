import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# 3D related Draw
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import warnings
import math


class RTPlotMatrix(object):
    def __init__(self):
        fig, ax = plt.subplots()

        self.DIMENSIONS = [1000,1001]

        N = np.random.random((self.DIMENSIONS[0], self.DIMENSIONS[1]))
        num_elemnts_matrix = N.shape[0]*N.shape[1]
        max_elem = math.pow(10,6)
        if num_elemnts_matrix > max_elem:
            print ("Warning, The Dimension is too high==>"+str(num_elemnts_matrix)+"> "+str(max_elem))
        self.matrice = ax.matshow(N)
        plt.colorbar(self.matrice)

        ani = animation.FuncAnimation(fig, self.update, frames=19, interval=100)
        plt.show()
        print "Matrix Ready"

    def update(self,i):
        N = np.random.random((self.DIMENSIONS[0], self.DIMENSIONS[1]))
        self.matrice.set_array(N)

class RTPlot3DMatrix(object):
    def __init__(self):
        fig = plt.figure()
        self.ax = fig.gca(projection='3d')
        self.X = np.arange(-5, 5, 0.1)
        self.Y = np.arange(-5, 5, 0.1)
        self.X, self.Y = np.meshgrid(self.X, self.Y)
        self.Z = np.random.rand(self.X.shape[0], self.Y.shape[0])

        self.surf = [self.ax.plot_surface(self.X, self.Y, self.Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)]

        print type(self.surf)
        print dir(self.surf)

        self.ax.set_zlim(-1.01, 1.01)
        self.ax.zaxis.set_major_locator(LinearLocator(10))
        self.ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

        fig.colorbar(self.surf[0], shrink=0.5, aspect=5)

        ani = animation.FuncAnimation(fig, self.update, frames=19, fargs=(self.surf), interval=100)
        plt.show()
        print "Matrix Ready"

    def update(self, frame_number, plot):
        self.surf[0].remove()
        self.Z = np.random.rand(self.X.shape[0], self.Y.shape[0])

        self.surf[0] = self.ax.plot_surface(self.X, self.Y, self.Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

if __name__ == "__main__":
    object = RTPlotMatrix()
    #object_3d = RTPlot3DMatrix()
