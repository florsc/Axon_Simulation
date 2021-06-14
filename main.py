from Simulation import Simulation
from StaticParameters import staticParametersDict

if __name__ == '__main__':
    simulation = Simulation(staticParametersDict)
    simulation.runSimulation()
    simulation.visualizeResults()