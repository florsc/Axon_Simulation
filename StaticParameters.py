import helperFunctions as hf

staticParametersDict = {
    "startingNumberOfAxons": 50,
    "alpha": 7.445,
    "beta": 1.665,
    # currently only a x value corresponding to the original simulation
    "targetAreaXValue": 100,
    "maximumNumberOfStepsEachTimePoint": 6,
    "growthLengthEachStep": 1,
    "axonDiameter": 0.4,
    "minimalDistanceBetweenStartingAxons": 0.4,
    "minimalDistanceBetweenAxons": 0.4,
    # starting area will be a cube with the axis limits given
    "startingArea": [(0, 0), (-20, 20), (-20, 20)],
    # I was absolutely not able to understand how this is handled in the original code, but it shouldn't have much influence anyway
    "initialThetas": [0, 0],
    "ExteriorLimit": hf.getExteriorLimitTube(33.4),
    "maximumNumberOfEncounters": 140,
    "probabilityForNewBranchEachTimeStep": 0.05
}
debugStuff = {
    "printInformation" : False,
    "measureTime" : True
}
