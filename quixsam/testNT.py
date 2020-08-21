from networktables import NetworkTables
import gtsam
from gtsam import symbol_shorthand_L as L
from gtsam import symbol_shorthand_X as X
import numpy as np
import time
import logging

def vector3(x, y, z):
	"""Create 3d double numpy array."""
	return np.array([x, y, z], dtype=np.float)

# As a client to connect to a robot
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize(server="host.docker.internal")

time.sleep(1)

PRIOR_NOISE = gtsam.noiseModel_Diagonal.Sigmas(vector3(1, 1, 1))
PRIOR_LANDMARK_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-6, 1e-6], dtype=np.float))
ODOMETRY_NOISE = gtsam.noiseModel_Diagonal.Sigmas(vector3(1e-1, 1e-1, 1e-1))
BEARING_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-2], dtype=np.float))

parameters = gtsam.ISAM2Params()
isam = gtsam.ISAM2(parameters)

graph = gtsam.NonlinearFactorGraph()
estimate = gtsam.Values()

rootTable = NetworkTables.getTable("Quixsam")
landmarks = rootTable.getSubTable('Landmarks')
estimates = rootTable.getSubTable('Estimates')

for i in landmarks.getKeys():
	landmark = landmarks.getNumberArray(i, [0, 0, 0])
	print(i)
	estimate.insert(L(landmark[0]), gtsam.Point2(landmark[1], landmark[2]))
	graph.add(gtsam.PriorFactorPoint2(L(landmark[0]), gtsam.Point2(landmark[1], landmark[2]), PRIOR_LANDMARK_NOISE))
	print(landmark)


def valueChanged(table, key, value, isNew):
	print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))

estimates.addEntryListener(valueChanged)
