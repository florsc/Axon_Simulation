import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


class Visualizer:

    def __init__(self, simulatedAxons):
        self.simulatedAxons = simulatedAxons
        plt.figure()
        self.axis = plt.axes(projection='3d')

    def addTubeLimits(self):

        # Cylinder
        x = np.linspace(0, 100, 100)
        z = np.linspace(-33.4, 33.4, 100)
        Xc, Zc = np.meshgrid(x, z)
        Yc = np.sqrt(33.4**2-Zc**2)

        # Draw parameters
        rstride = 20
        cstride = 10
        self.axis.plot_surface(Xc, Yc, Zc, color='r',
                               alpha=0.2, rstride=rstride, cstride=cstride)
        self.axis.plot_surface(Xc, -Yc, Zc, color='r',
                               alpha=0.2, rstride=rstride, cstride=cstride)

        self.axis.set_xlabel("X")
        self.axis.set_ylabel("Y")
        self.axis.set_zlabel("Z")

    def addAxons(self):
        for axon in self.simulatedAxons:
            xline = [tipPosition[0]
                     for tipPosition in axon["axonalTipPositions"]]
            yline = [tipPosition[1]
                     for tipPosition in axon["axonalTipPositions"]]
            zline = [tipPosition[2]
                     for tipPosition in axon["axonalTipPositions"]]
            self.axis.plot3D(xline, yline, zline, 'gray')

    def visualize(self, exteriorLimit):
        self.addAxons()
        if exteriorLimit == "TUBE":
            self.addTubeLimits()
        plt.show()
        pass
