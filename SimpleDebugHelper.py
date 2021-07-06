import time
from StaticParameters import debugStuff

class SimpleDebugHelper:
	def __init__(self):
		self.printInformation = debugStuff["printInformation"]
		self.measureTime = debugStuff["measureTime"]
		self.measuredObjects = {}
		self.startPoints = {}

	def start(self, title):
		if self.measureTime:
			self.startPoints[title] = time.time()
		
	def stop(self, title):
		end = time.time()
		if title in self.startPoints:
			end = time.time()
			if self.startPoints[title] != 0:
				if title in self.measuredObjects:
					if self.measuredObjects[title]:
						self.measuredObjects[title]["runtime"] += end - self.startPoints[title]
						self.measuredObjects[title]["number"] += 1

				else:
					self.measuredObjects[title] = {"runtime": end-self.startPoints[title], "number" :1}
		self.startPoints[title] = 0

	def print(self, info):
		if self.printInformation:
			print(info)

	def printMeasurements(self):
		print("Measurements:")
		for title, info in self.measuredObjects.items():
			print(title + ": complete runtime is "+ str(info["runtime"]) +", average runtime is "+ str(info["runtime"]/info["number"]) + ", number of calls is "+ str(info["number"]))