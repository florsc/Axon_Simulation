import numpy as np

class Simulation:
  def __init__(self,staticModelParametersDictionary):
    self.staticParameters = dict(staticModelParametersDictionary)
    print("Set static model parameters:")
    print(self.staticParameters)
    self.initializeVariables()

  def initializeVariables(self):
    self.currentTimeStep = 0
    self.numberOfAxons = self.staticParameters["startingNumberOfAxons"]
    self.numberOfFinishedAxons=0

  
  def runSimulation(self):
    while (self.numberOfAxons - self.numberOfFinishedAxons) > 0:
  		
        self.currentTimeStep += 1

        self.numberOfFinishedAxons+=1 #will be removed

  def visualizeResults(self):
    #TODO
    pass