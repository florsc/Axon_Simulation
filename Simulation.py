import numpy as np
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
                startingPositions[i])], "thetas": [self.staticParameters["initialThetas"]], "interactionCounter": 0, "active": True})

            print("Init axon with starting point: " +
                  str(startingPositions[i]))

        # TODO this whole spatial occupation stuff is most likely possible to implement more efficient with numpy and the correct functions
        self.occupiedSpatialAreaCenters = hf.sortNumpyArrayAlongColumn(
            np.array(startingPositions), 0)
        self.numberOfAreaCentersEachGrowthStep = math.floor(
            self.staticParameters["growthLengthEachStep"]/self.staticParameters["minimalDistanceBetweenAxons"])
        self.distanceBetweenCenters = self.staticParameters["minimalDistanceBetweenAxons"] / \
            self.numberOfAreaCentersEachGrowthStep

    def getStepSize(self):
        # can be randomized but then self.numberOfAreaCentersEachGrowthStep needs to be adjusted
        return self.staticParameters["growthLengthEachStep"]

    def sampleModelGrowthVector(self, currentAxon, growthStepThisTimePoint):
        alpha = self.staticParameters["alpha"]
        beta = self.staticParameters["beta"]
        step = growthStepThisTimePoint

        lastTheta_0 = currentAxon["thetas"][-1][0]
        expectation_0 = (alpha/(alpha+beta))*lastTheta_0
        variation_0 = 1/math.sqrt(2*(alpha+beta))
        newTheta_0 = np.random.normal(expectation_0, variation_0)

        lastTheta_1 = currentAxon["thetas"][-1][1]
        expectation_1 = (alpha/(alpha+beta))*lastTheta_1
        variation_1 = 1/math.sqrt(2*(alpha+beta))
        newTheta_1 = np.random.normal(expectation_1, variation_1)

        # in the original simulation the external field is added here
        # possibly TODO

        finalAngle_0 = 2*np.arctan(lastTheta_0)
        finalAngle_1 = 2*np.arctan(lastTheta_1)

        growthVector = np.array(hf.sph2cart(
            finalAngle_0, finalAngle_1, self.getStepSize()))
        return growthVector, (newTheta_0, newTheta_1)

    def addOccupiedAreaCenters(self, currentAxon, growthStepThisTimePoint):
        newSpatialAreaCenters = []
        tipPositions = currentAxon["axonalTipPositions"]
        for growthStep in range(growthStepThisTimePoint):
            for center in range(self.numberOfAreaCentersEachGrowthStep-1):
                newSpatialAreaCenters.append(tipPositions[growthStep]+center*self.distanceBetweenCenters*(
                    tipPositions[growthStep]-tipPositions[growthStep-1]))

        self.occupiedSpatialAreaCenters = hf.sortNumpyArrayAlongColumn(
            np.append(self.occupiedSpatialAreaCenters, newSpatialAreaCenters, 0), 1)

    def handleMechanicalInteraction(self, currentAxon, growthStepThisTimePoint):
        for i in range(min(2, growthStepThisTimePoint)):
            currentAxon["thetas"].pop()
            currentAxon["axonalTipPositions"].pop()
        pass

    def runTimeStepForAxon(self, currentAxon):
        growthStepThisTimePoint = 1
        interactionCounter = 0

        while growthStepThisTimePoint < self.staticParameters["maximumNumberOfStepsEachTimePoint"] and interactionCounter < 2:
            growthVector, sampledTheta = self.sampleModelGrowthVector(currentAxon,
                                                                      growthStepThisTimePoint)
            # TODO this could cause issues when retraction is implemented, replace -1
            currentAxon["axonalTipPositions"].append(
                growthVector + currentAxon["axonalTipPositions"][-1])
            currentAxon["thetas"].append(sampledTheta)

            if self.checkForMechanicalInteractions(currentAxon):
                print("Mechanical Interaction encountered at position: " +
                      str(currentAxon["axonalTipPositions"][-1]))
                print("Number of interactions this timePoint: " +
                      str(interactionCounter))
                self.handleMechanicalInteraction(
                    currentAxon, growthStepThisTimePoint)
                growthStepThisTimePoint = max(growthStepThisTimePoint-1, 1)
                interactionCounter += 1
            else:
                growthStepThisTimePoint += 1

        currentAxon["interactionCounter"] += interactionCounter
        self.addOccupiedAreaCenters(currentAxon, growthStepThisTimePoint)

    def checkForOtherNeurites(self, currentAxon):
        otherNeuriteEncountered = False
        if len(self.occupiedSpatialAreaCenters) > 0:
            tipPosition = currentAxon["axonalTipPositions"][-1]
            # TODO leverage the order in occupiedSpatialAreaCenters
            if np.any(np.argwhere(np.linalg.norm(self.occupiedSpatialAreaCenters - tipPosition) <= self.staticParameters["minimalDistanceBetweenAxons"])):
                otherNeuriteEncountered = True

        return otherNeuriteEncountered

    def checkForExteriorLimits(self, currentAxon):
        HELLO = False  # A better name would be exteriorLimitEncountered, but to honor the most random variable name I have ever seen I decided to use the name from the original code
        position = currentAxon["axonalTipPositions"][-1]
        limitDict = self.staticParameters["ExteriorLimit"]
        if limitDict["type"] == "TUBE":
            if position[1] ** 2 + position[2] ** 2 >= limitDict["radius"] ** 2:
                HELLO = True

        return HELLO

    def checkForMechanicalInteractions(self, currentAxon):
        mechanicalConstraintEncountered = self.checkForExteriorLimits(
            currentAxon)
        if not mechanicalConstraintEncountered:
            mechanicalConstraintEncountered = self.checkForOtherNeurites(
                currentAxon)

        return mechanicalConstraintEncountered

    # TODO stop all branches when branching is implemented
    def checkTargetReached(self, currentAxon):
        if currentAxon["axonalTipPositions"][-1][0] > self.staticParameters["targetAreaXValue"]:
            print("Axon reached finish at position." +
                  str(currentAxon["axonalTipPositions"][-1]))
            currentAxon["active"] = False
            self.numberOfFinishedAxons += 1

    def getAxisStartPositions(self, axisIndex):
        if self.staticParameters["startingArea"][axisIndex][0] == self.staticParameters["startingArea"][axisIndex][1]:
            return [self.staticParameters["startingArea"][axisIndex][0]]
        else:
            spacePerAxon = self.staticParameters["axonDiameter"] + \
                self.staticParameters["minimalDistanceBetweenStartingAxons"]
            axisLength = self.staticParameters["startingArea"][axisIndex][1] - \
                self.staticParameters["startingArea"][axisIndex][0]
            numberOfPositions = math.floor(axisLength/spacePerAxon)
            remainingSpaceAtBorder = (
                axisLength-(numberOfPositions*spacePerAxon))/2
            axisPositions = [remainingSpaceAtBorder + self.staticParameters["startingArea"][axisIndex][0] +
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

    def checkNumberOfEncounters(self, axon):
        if axon["interactionCounter"] > self.staticParameters["maximumNumberOfEncounters"]:
            axon["active"] = False
            self.numberOfFinishedAxons += 1

    def runSimulation(self):
        while (self.numberOfAxons - self.numberOfFinishedAxons) > 0:
            print("Timestep: " + str(self.currentTimeStep))
            print("Current axon tip points:")
            for axonIndex in range(len(self.axons)):
                print("Axon " + str(axonIndex) + "(" + ("active" if self.axons[axonIndex]["active"] else "finished") + ": " + str(
                    self.axons[axonIndex]["axonalTipPositions"][-1]))
            for axon in self.axons:
                if axon["active"] == True:

                    self.runTimeStepForAxon(axon)
                    self.checkNumberOfEncounters(axon)
                    self.checkTargetReached(axon)

                    self.currentTimeStep += 1

        print("Simulation successfull finished.")
