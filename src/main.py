import gtsam
from gtsam import symbol_shorthand_L as L
from gtsam import symbol_shorthand_X as X
import numpy as np
import time

np.random.seed(0)


def vector3(x, y, z):
	"""Create 3d double numpy array."""
	return np.array([x, y, z], dtype=np.float)


NUM_NODES = 3 * 60 * 50  # 3 minutes at 50 Hz
PRIOR_NOISE = gtsam.noiseModel_Diagonal.Sigmas(vector3(1, 1, 1))
PRIOR_LANDMARK_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-6, 1e-6], dtype=np.float))
ODOMETRY_NOISE = gtsam.noiseModel_Diagonal.Sigmas(vector3(1e-1, 1e-1, 1e-1))
BEARING_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.array([1e-2], dtype=np.float))

parameters = gtsam.ISAM2Params()
#parameters.setRelinearizeThreshold(0.01)
#parameters.setRelinearizeSkip(1)
isam = gtsam.ISAM2(parameters)

graph = gtsam.NonlinearFactorGraph()
estimate = gtsam.Values()

# Add landmarks
estimate.insert(L(0), gtsam.Point2(10, 0))
graph.add(gtsam.PriorFactorPoint2(L(0), gtsam.Point2(10, 0), PRIOR_LANDMARK_NOISE))
estimate.insert(L(1), gtsam.Point2(10, 2))
graph.add(gtsam.PriorFactorPoint2(L(1), gtsam.Point2(10, 2), PRIOR_LANDMARK_NOISE))


simulated_pose = gtsam.Pose2()

# Begin
for i in range(0, NUM_NODES):
	s = time.time()

	# Add new pose estimate
	estimate.insert(X(i), simulated_pose)

	# Add odometry factor
	if i == 0:
		graph.add(gtsam.PriorFactorPose2(X(0), simulated_pose, PRIOR_NOISE))
	else:
		graph.add(gtsam.BetweenFactorPose2(X(i-1), X(i), gtsam.Pose2(1 + np.random.normal(0, 0.05), 0, np.pi/4 + np.random.normal(0, 0.05)), ODOMETRY_NOISE))

	# Add landmark factor
	if i % 8 == 0:
		graph.add(gtsam.BearingFactor2D(X(i), L(0), gtsam.Rot2(0), BEARING_NOISE))
		graph.add(gtsam.BearingFactor2D(X(i), L(1), gtsam.Rot2.fromDegrees(11.3), BEARING_NOISE))
	if i % 8 == 1:
		graph.add(gtsam.BearingFactor2D(X(i), L(0), gtsam.Rot2.fromDegrees(-45.0), BEARING_NOISE))
		graph.add(gtsam.BearingFactor2D(X(i), L(1), gtsam.Rot2.fromDegrees(-32.5), BEARING_NOISE))
	if i % 8 == 2:
		graph.add(gtsam.BearingFactor2D(X(i), L(1), gtsam.Rot2.fromDegrees(-81.1), BEARING_NOISE))
	if i % 8 == 7:
		graph.add(gtsam.BearingFactor2D(X(i), L(0), gtsam.Rot2.fromDegrees(41.2), BEARING_NOISE))
		#graph.add(gtsam.BearingFactor2D(X(i), L(1), gtsam.Rot2.fromDegrees(51.9), BEARING_NOISE))

	# Simulate motion
	simulated_pose = simulated_pose.compose(gtsam.Pose2(1, 0, np.pi/4))

	# Solve incremental
	if i % 50 == 0:
		isam.update(graph, estimate)
		graph.resize(0)
		estimate.clear()

		current_estimate = isam.calculateEstimate()
		#marginals = gtsam.Marginals(isam.getFactorsUnsafe(), current_estimate)

		print(f'{i}: {time.time() - s}')

		if i % 8 == 0:
			print(current_estimate.atPose2(X(i)))
