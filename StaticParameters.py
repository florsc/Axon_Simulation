staticParametersDict = {
    "startingNumberOfAxons": 1,
    "alpha": 7.445,
    "beta": 1.665,
    # currently only a x value corresponding to the original simulation
    "targetAreaXValue": 70,
    "maximumNumberOfStepsEachTimePoint": 6,
    "stepSize": 1,
    "axonDiameter": 0.4,
    # starting area will be a cube with the axis limits given
    "startingArea": [(0, 0), (0, 100), (0, 100)],
    # I was absolutely not able to understand how this is handled in the original code, but it shouldn't have much influence anyway
    "initialThetas": [0, 0]
}
