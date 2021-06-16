import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import math
import helperFunctions as hf


class Simulation:
    def __init__(self, staticModelParametersDictionary):
        self.staticParameters = dict(staticModelParametersDictionary)
        print("Set static model parameters:")
        print(self.staticParameters)
        self.initializeVariables()

    def initializeVariables(self):
        self.currentTimeStep = 0
        self.numberOfAxons = self.staticParameters["startingNumberOfAxons"]
        self.numberOfFinishedAxons = 0
        self.axons = []
        startingPositions = self.getStartPositions()
        for i in range(self.staticParameters["startingNumberOfAxons"]):
            self.axons.append({"counter": 0, "axonalTipPositions": [np.array(
                startingPositions[i])], "lastTheta": self.staticParameters["initialThetas"]})
            print("Init axon with starting point: " +
                  str(startingPositions[i]))

    def getStepSize(self):
        return self.staticParameters["stepSize"]  # can be randomized

    def sampleModelGrowthVector(self, growthStepThisTimePoint):
        alpha = self.staticParameters["alpha"]
        beta = self.staticParameters["beta"]
        step = growthStepThisTimePoint

        thetas_0 = self.thetasThisTimePoint[0]
        expectation_0 = (alpha/(alpha+beta))*thetas_0[step-1]
        variation_0 = 1/math.sqrt(2*(alpha+beta))
        # this modifies self.thetas
        thetas_0.append(np.random.normal(expectation_0, variation_0))

        thetas_1 = self.thetasThisTimePoint[1]
        expectation_1 = (alpha/(alpha+beta))*thetas_1[step-1]
        variation_1 = 1/math.sqrt(2*(alpha+beta))
        # this modifies self.thetas
        thetas_1.append(np.random.normal(expectation_1, variation_1))

        # in the original simulation the external field is added here
        # possibly TODO

        finalAngle_0 = 2*np.arctan(thetas_0[step])
        finalAngle_1 = 2*np.arctan(thetas_1[step])

        growthVector = np.array(hf.sph2cart(
            finalAngle_0, finalAngle_1, self.getStepSize()))
        return growthVector

    def runTimeStepForAxon(self, currentAxon):
        self.thetasThisTimePoint = [
            [currentAxon["lastTheta"][0]], [currentAxon["lastTheta"][1]]]
        growthStepThisTimePoint = 1

        while growthStepThisTimePoint < self.staticParameters["maximumNumberOfStepsEachTimePoint"]:
            growthVector = self.sampleModelGrowthVector(
                growthStepThisTimePoint)
            # TODO this could cause issues when retraction is implemented, replace -1
            currentAxon["axonalTipPositions"].append(
                growthVector + currentAxon["axonalTipPositions"][-1])
            growthStepThisTimePoint += 1

    def checkTargetReached(self, axon):
        # TODO
        pass

    def getAxisStartPositions(self, axisIndex):
        if self.staticParameters["startingArea"][axisIndex][0] == self.staticParameters["startingArea"][axisIndex][1]:
            return [self.staticParameters["startingArea"][axisIndex][0]]
        else:
            spatialStartingCubeFactor = 2
            spacePerAxon = self.staticParameters["axonDiameter"] * \
                spatialStartingCubeFactor
            axisLength = self.staticParameters["startingArea"][axisIndex][1] - \
                self.staticParameters["startingArea"][axisIndex][0]
            numberOfPositions = math.floor(axisLength/spacePerAxon)
            remainingSpaceAtBorder = (
                axisLength-(numberOfPositions*spacePerAxon))/2
            axisPositions = [remainingSpaceAtBorder +
                             (p * spacePerAxon) for p in range(numberOfPositions)]
            return axisPositions

    def getStartPositions(self):
        """The sampling process in the original code was wild and highly problem oriented.
        So I will create a simple placeholder here and hope we have different requirements anyway."""
        xAxisPositions = self.getAxisStartPositions(0)
        yAxisPositions = self.getAxisStartPositions(1)
        zAxisPositions = self.getAxisStartPositions(2)
        numberOfPositions = len(xAxisPositions) * \
            len(yAxisPositions) * len(zAxisPositions)
        if numberOfPositions < self.staticParameters["startingNumberOfAxons"]:
            assert False, "More starting axons than starting positions. This could be a good place for a pun about beeing too dense."

        linearPositionIndices = np.random.choice(
            numberOfPositions, self.staticParameters["startingNumberOfAxons"], replace=False)
        lenX = len(xAxisPositions)
        lenY = len(yAxisPositions)
        lenZ = len(zAxisPositions)
        xPositionsIndices = [math.floor(
            linearPositionIndices[i] / (lenY*lenZ)) for i in range(len(linearPositionIndices))]
        yPositionsIndices = [math.floor((linearPositionIndices[i] - (
            lenX * xPositionsIndices[i])) / lenZ) for i in range(len(linearPositionIndices))]
        zPositionsIndices = [math.floor((linearPositionIndices[i] - (lenX * xPositionsIndices[i]) - (
            lenY * yPositionsIndices[i]))) for i in range(len(linearPositionIndices))]
        positions = [[xAxisPositions[xPositionsIndices[i]], yAxisPositions[yPositionsIndices[i]],
                      zAxisPositions[zPositionsIndices[i]]] for i in range(self.staticParameters["startingNumberOfAxons"])]
        return positions

    def runSimulation(self):
        while (self.numberOfAxons - self.numberOfFinishedAxons) > 0:
            print("Timestep: " + str(self.currentTimeStep))
            print("Current axon tip points:")
            for axon in self.axons:
                print(axon["axonalTipPositions"][-1])
            for axon in self.axons:

                self.runTimeStepForAxon(axon)
                self.checkTargetReached(axon)

                self.currentTimeStep += 1

                # will be removed, currently only to ensure ending of loop
                self.numberOfFinishedAxons += 0.1

        print("Simulation successfull finished.")

    def visualizeResults(self):
        # currently simple visualization for the first axon to check result
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        xline = [tipPosition[0]
                 for tipPosition in self.axons[0]["axonalTipPositions"]]
        yline = [tipPosition[1]
                 for tipPosition in self.axons[0]["axonalTipPositions"]]
        zline = [tipPosition[2]
                 for tipPosition in self.axons[0]["axonalTipPositions"]]
        ax.plot3D(xline, yline, zline, 'gray')
        plt.show()
        pass
