from Simulation import Simulation
from Visualizer import Visualizer
from StaticParameters import staticParametersDict
from SimpleDebugHelper import SimpleDebugHelper

if __name__ == '__main__':
    simpleDebugHelper = SimpleDebugHelper()
    simpleDebugHelper.start("complete")
    simulation = Simulation(staticParametersDict, simpleDebugHelper)
    simulation.runSimulation()
    simpleDebugHelper.stop("complete")
    simpleDebugHelper.printMeasurements()
    visualizer = Visualizer(simulation.axons)
    visualizer.visualize(staticParametersDict["ExteriorLimit"],staticParametersDict["target"])
