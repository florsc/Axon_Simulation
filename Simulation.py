import numpy as np
import math
import helperFunctions as hf
from scipy.stats import levy_stable


class Simulation:
    def __init__(self, staticModelParametersDictionary, simpleDebugHelper):
        self.staticParameters = dict(staticModelParametersDictionary)
        self.simpleDebugHelper = simpleDebugHelper
        self.simpleDebugHelper.print("Set static model parameters:")
        self.simpleDebugHelper.print(self.staticParameters)
        self.initializeVariables()

    def sampleInitialThetas(self):
        return np.tan(np.random.uniform(-math.pi,math.pi,2))

    def initializeVariables(self):
        self.currentTimeStep = 0
        self.numberOfAxons = self.staticParameters["startingNumberOfAxons"]
        self.numberOfFinishedAxons = 0
        self.axons = []
        startingPositions = self.getStartPositions()
        for i in range(self.staticParameters["startingNumberOfAxons"]):
            self.axons.append({"counter": 0, "axonalTipPositions": [np.array(
                startingPositions[i])], "thetas": [self.sampleInitialThetas()], "interactionCounter": 0, "active": True, "rootAxonIndex" : i, "branchIndices" : []})

            self.simpleDebugHelper.print("Init axon with starting point: " +
                  str(startingPositions[i]))
        self.occupiedSpatialAreaCenters= np.array(startingPositions)[np.array(startingPositions)[:, 0].argsort()]

        
    def getStepSize(self):
        # can be randomized but then self.numberOfAreaCentersEachGrowthStep needs to be adjusted
        stepLengthData = self.staticParameters["growthLengthEachStep"]
        if stepLengthData["type"] == "LEVY":
            return levy_stable.rvs(stepLengthData["alpha"], stepLengthData["beta"])
        if stepLengthData["type"] == "CONSTANT":
            return stepLengthData["length"]

    def sampleModelGrowthVector(self, currentAxon):
        alpha = self.staticParameters["alpha"]
        beta = self.staticParameters["beta"]

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

        stepSize = self.getStepSize()
        growthVector = np.array(hf.sph2cart(
            finalAngle_0, finalAngle_1, stepSize))
        return growthVector, (newTheta_0, newTheta_1)

    def addOccupiedAreaCenters(self, currentAxon, growthStepThisTimePoint):
        newSpatialAreaCenters = []
        tipPositions = currentAxon["axonalTipPositions"]
        for growthStep in range(growthStepThisTimePoint):
            stepSize = np.linalg.norm(tipPositions[growthStep] - tipPositions[growthStep-1])
            numberOfAreaCentersEachGrowthStep = math.ceil(
                stepSize/self.staticParameters["minimalDistanceBetweenAxons"])
            distanceBetweenCenters = stepSize / \
                numberOfAreaCentersEachGrowthStep
            for centerNumber in range(numberOfAreaCentersEachGrowthStep):
                newSpatialAreaCenters.append(tipPositions[growthStep]+centerNumber*distanceBetweenCenters*(
                    tipPositions[growthStep]-tipPositions[growthStep-1]))
        self.occupiedSpatialAreaCenters = hf.addNumpyArraySortedAlongColumn(
            self.occupiedSpatialAreaCenters, np.array(newSpatialAreaCenters), 0)

    def handleMechanicalInteraction(self, currentAxon, growthStepThisTimePoint):
        for i in range(min(2, growthStepThisTimePoint)):
            currentAxon["thetas"].pop()
            currentAxon["axonalTipPositions"].pop()

    def checkNewBranchCondition(self):
        if np.random.uniform(0,1)<self.staticParameters["probabilityForNewBranchEachTimeStep"]:
            return True

    def createNewBranch(self, branchingAxon, possibleBranchingPoints):
        branchingPoint = branchingAxon["axonalTipPositions"][-np.random.randint(possibleBranchingPoints)]
        branchingAxon["branchIndices"].append(len(self.axons))
        self.axons.append({"axonalTipPositions": [branchingPoint], "thetas": [self.staticParameters["initialThetas"]], "interactionCounter": branchingAxon["interactionCounter"], "active": True, "rootAxonIndex": branchingAxon["rootAxonIndex"], "branchIndices": []})


    def runTimeStepForAxon(self, currentAxon):
        growthStepThisTimePoint = 1
        interactionCounter = 0

        while growthStepThisTimePoint <= self.staticParameters["maximumNumberOfStepsEachTimePoint"] and interactionCounter < 2:
            growthVector, sampledTheta = self.sampleModelGrowthVector(currentAxon)
            currentAxon["axonalTipPositions"].append(
                growthVector + currentAxon["axonalTipPositions"][-1])
            currentAxon["thetas"].append(sampledTheta)

            if self.checkForMechanicalInteractions(currentAxon):
                self.simpleDebugHelper.print("Mechanical Interaction encountered at position: " +
                      str(currentAxon["axonalTipPositions"][-1]))
                self.simpleDebugHelper.print("Number of interactions this timePoint: " +
                      str(interactionCounter))
                self.handleMechanicalInteraction(
                    currentAxon, growthStepThisTimePoint)
                growthStepThisTimePoint = max(growthStepThisTimePoint-1, 1)
                interactionCounter += 1
            else:
                growthStepThisTimePoint += 1

        if self.checkNewBranchCondition():
            self.createNewBranch(currentAxon, growthStepThisTimePoint)

        currentAxon["interactionCounter"] += interactionCounter
        if growthStepThisTimePoint>1:
            self.addOccupiedAreaCenters(currentAxon, growthStepThisTimePoint)

    def checkForOtherNeurites(self, currentAxon):
        otherNeuriteEncountered = False
        if len(self.occupiedSpatialAreaCenters) > 0:
            newPosition = currentAxon["axonalTipPositions"][-1]
            oldPosition = currentAxon["axonalTipPositions"][-2]
            difference = oldPosition-newPosition
            stepSize = np.linalg.norm(newPosition - oldPosition)
            numberOfAreaCentersGrowthStep = math.floor(
                stepSize/self.staticParameters["minimalDistanceBetweenAxons"])+1

            centers = np.linspace(oldPosition,newPosition,numberOfAreaCentersGrowthStep)
            for i in range(numberOfAreaCentersGrowthStep):
                self.simpleDebugHelper.start("checkForOtherNeurites")
                lowHigh = np.searchsorted(self.occupiedSpatialAreaCenters[:,0],[centers[i][0]-self.staticParameters["minimalDistanceBetweenAxons"], centers[i][0]+self.staticParameters["minimalDistanceBetweenAxons"]])
                if np.any(np.argwhere(np.linalg.norm(self.occupiedSpatialAreaCenters[lowHigh[0]:lowHigh[1]] - centers[i]) <= self.staticParameters["minimalDistanceBetweenAxons"])):
                    otherNeuriteEncountered = True
                    break

            self.simpleDebugHelper.stop("checkForOtherNeurites")
        return otherNeuriteEncountered

    def checkForExteriorLimits(self, currentAxon):
        exteriorLimitEncountered = False  
        position = currentAxon["axonalTipPositions"][-1]
        limitDict = self.staticParameters["ExteriorLimit"]
        if limitDict["type"] == "TUBE":
            if position[1] ** 2 + position[2] ** 2 >= limitDict["radius"] ** 2:
                exteriorLimitEncountered = True

        if limitDict["type"] == "BALL":
            if position[0] ** 2 + position[1] ** 2 + position[2] ** 2>= limitDict["radius"] ** 2:
                exteriorLimitEncountered = True

        return exteriorLimitEncountered

    def checkForMechanicalInteractions(self, currentAxon):
        mechanicalConstraintEncountered = self.checkForExteriorLimits(
            currentAxon)
        if not mechanicalConstraintEncountered:
            mechanicalConstraintEncountered = self.checkForOtherNeurites(
                currentAxon)

        return mechanicalConstraintEncountered

    def finishAxonAndBranches(self, axon):
        for i in axon["branchIndices"]:
            self.finishAxonAndBranches(self.axons[i])
        self.finishAxonBranch(axon)

    def finishAxonBranch(self, axon):
        if axon["active"]:
            axon["active"] = False
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
            (lenY*lenZ) * xPositionsIndices[i])) / lenZ) for i in range(len(linearPositionIndices))]
        zPositionsIndices = [math.floor((linearPositionIndices[i] - ((lenY*lenZ) * xPositionsIndices[i]) - (
            lenZ * yPositionsIndices[i]))) for i in range(len(linearPositionIndices))]
        positions = [[xAxisPositions[xPositionsIndices[i]], yAxisPositions[yPositionsIndices[i]],
                      zAxisPositions[zPositionsIndices[i]]] for i in range(self.staticParameters["startingNumberOfAxons"])]
        return positions

    def checkNumberOfEncounters(self, axon):
        if axon["interactionCounter"] > self.staticParameters["maximumNumberOfEncounters"]:
            self.finishAxonBranch(axon)

    def checkTargetReached(self, currentAxon):
        distanceVectors = currentAxon["axonalTipPositions"][-1] - self.staticParameters["target"]["centers"]

        if np.any( distanceVectors[:,0] ** 2 +distanceVectors[:,1] ** 2 +distanceVectors[:,2] ** 2 < self.staticParameters["target"]["radius"]**2):
            self.simpleDebugHelper.print("Axon reached finish at position." +
                  str(currentAxon["axonalTipPositions"][-1]))
            self.finishAxonAndBranches(self.axons[currentAxon["rootAxonIndex"]])

    def runSimulation(self):
        while (self.numberOfAxons - self.numberOfFinishedAxons) > 0:

            self.simpleDebugHelper.print("Timestep: " + str(self.currentTimeStep))
            self.simpleDebugHelper.print("Current axon tip points:")

            for axonIndex in range(len(self.axons)):
                self.simpleDebugHelper.print("Axon " + str(axonIndex) + "(" + ("active" if self.axons[axonIndex]["active"] else "finished") + ": " + str(
                    self.axons[axonIndex]["axonalTipPositions"][-1]))
            for axon in self.axons:
                if axon["active"] == True:

                    self.runTimeStepForAxon(axon)
                    self.checkNumberOfEncounters(axon)
                    self.checkTargetReached(axon)

                    self.currentTimeStep += 1

        self.simpleDebugHelper.print("Simulation successfull finished.")
