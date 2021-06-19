from Simulation import Simulation
from Visualizer import Visualizer
from StaticParameters import staticParametersDict

if __name__ == '__main__':
    simulation = Simulation(staticParametersDict)
    simulation.runSimulation()
    visualizer = Visualizer(simulation.axons)
    visualizer.visualize(exteriorLimit="TUBE")
