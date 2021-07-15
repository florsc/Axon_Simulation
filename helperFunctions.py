import numpy as np

def addNumpyArraySortedAlongColumn(sortedArray, addArray, columnNumber):
    addArraySorted = np.array(addArray)[np.array(addArray)[:, 0].argsort()]
    newIndices = np.searchsorted(sortedArray[:, columnNumber],addArraySorted[:,columnNumber])
    return np.insert(sortedArray,newIndices ,addArraySorted, axis=0)



def sph2cart(az, el, r):
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return [x, y, z]

def getTargetBalls(centers, radius):
    ballsDict = {"type": "BALL", "centers": np.array(centers), "radius": radius}
    return ballsDict

def getExteriorLimitTube(radius):
    tubeLimitDictionary = {"type": "TUBE", "radius": radius}
    return tubeLimitDictionary

def getExteriorLimitBall(radius):
    ballLimitDictionary = {"type": "BALL", "radius": radius}
    return ballLimitDictionary

def getStepLengthConstant(length):
    data = {"type": "CONSTANT", "length": length}
    return data

def getStepLengthLevyFlights(alpha, beta):
    data = {"type": "LEVY", "alpha": alpha, "beta": beta}
    return data
