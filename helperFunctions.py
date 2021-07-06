import numpy as np


def sortNumpyArrayAlongColumn(array, columnNumber):
    # TODO in our usecase an inplace insertion sort would be ideal, maybe later
    return array #array[array[:, 0].argsort()]


def sph2cart(az, el, r):
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return [x, y, z]


def getExteriorLimitTube(radius):
    tubeLimitDictionary = {"type": "TUBE", "radius": radius}
    return tubeLimitDictionary
