import helperFunctions as hf

staticParametersDict = {
    "startingNumberOfAxons": 10,
    "alpha": 7.445,
    "beta": 1.665,
    # currently only a x value corresponding to the original simulation
    "target": hf.getTargetBalls([[5,5,5], [2,1,-3]], 1),
    "maximumNumberOfStepsEachTimePoint": 6,
    "growthLengthEachStep": hf.getStepLengthConstant(1),
    "axonDiameter": 0.4,
    "minimalDistanceBetweenStartingAxons": 0.4,
    "minimalDistanceBetweenAxons": 0.4,
    # starting area will be a cube with the axis limits given
    "startingArea": [(-10, 10), (-10, 10), (-10, 10)],
    # I was absolutely not able to understand how this is handled in the original code, but it shouldn't have much influence anyway
    "initialThetas": [0, 0],
    "ExteriorLimit": hf.getExteriorLimitBall(15),
    "maximumNumberOfEncounters": 140,
    "probabilityForNewBranchEachTimeStep": 0.01
}
debugStuff = {
    "printInformation" : False,
    "measureTime" : False
}
