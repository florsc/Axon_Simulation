import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


class Visualizer:

    def __init__(self, simulatedAxons):
        self.simulatedAxons = simulatedAxons
        plt.figure()
        self.axis = plt.axes(projection='3d')

    def addBallLimits(self, radius):

        # Ball
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = radius * np.outer(np.cos(u), np.sin(v))
        y = radius * np.outer(np.sin(u), np.sin(v))
        z = radius * np.outer(np.ones(np.size(u)), np.cos(v))

        # Plot the surface
        self.axis.plot_surface(x, y, z, alpha=0.2, color='b') 

        


    def addTubeLimits(self, radius):

        # Cylinder
        x = np.linspace(0, 100, 10)
        z = np.linspace(-33.4, 33.4, 10)
        Xc, Zc = np.meshgrid(x, z)
        Yc = np.sqrt(33.4**2-Zc**2)

        # Draw parameters
        rstride = 20
        cstride = 10
        self.axis.plot_surface(Xc, Yc, Zc, color='r',
                               alpha=0.2, rstride=rstride, cstride=cstride)
        self.axis.plot_surface(Xc, -Yc, Zc, color='r',
                               alpha=0.2, rstride=rstride, cstride=cstride)



    def addTargets(self, targets):
        radius = targets["radius"]
        print(targets["centers"].size)
        for index in range(targets["centers"][:,0].size):
            center = targets["centers"][index]
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = radius * np.outer(np.cos(u), np.sin(v))+center[0]
            y = radius * np.outer(np.sin(u), np.sin(v))+center[1]
            z = radius * np.outer(np.ones(np.size(u)), np.cos(v))+center[2]

            # Plot the surface
            self.axis.plot_surface(x, y, z, alpha=0.2, color='r') 

    def addAxons(self):
        for i in range(len(self.simulatedAxons)):
            axon=self.simulatedAxons[i]
            tipPositions = axon["axonalTipPositions"]
            if axon["rootAxonIndex"] == i:
                self.axis.plot3D([tipPositions[0][0]],[tipPositions[0][1]],[tipPositions[0][2]], 'ys',markersize=10) 
            else:
                self.axis.plot3D([tipPositions[0][0]],[tipPositions[0][1]],[tipPositions[0][2]], 'gs',markersize=5) 
            xline = [tipPosition[0]
                     for tipPosition in axon["axonalTipPositions"]]
            yline = [tipPosition[1]
                     for tipPosition in axon["axonalTipPositions"]]
            zline = [tipPosition[2]
                     for tipPosition in axon["axonalTipPositions"]]
            self.axis.plot3D(xline, yline, zline, 'gray')

    def visualize(self, exteriorLimit,targets):
        self.addAxons()
        if exteriorLimit["type"] == "TUBE":
            self.addTubeLimits()
        if exteriorLimit["type"] == "BALL":
            self.addBallLimits(exteriorLimit["radius"])
        self.axis.set_xlabel("X")
        self.axis.set_ylabel("Y")
        self.axis.set_zlabel("Z")


        self.addTargets(targets)
        plt.show()
        pass
