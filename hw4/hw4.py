#!/usr/bin/python
import scipy
import main
import plot
import ekf_slam

# This is the main extended Kalman filter SLAM loop. This script calls all the required
# functions in the correct order.
#
# You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should however not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions you
# should read their documentation which tells you the expected dimensions.

# Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = main.read_world('../world.dat')
# load landmarks;
# Read sensor readings, i.e. odometry and range-bearing sensor
data = main.read_data('../sensor_data.dat')
#load data

infty = 1000.
# Get the number of landmarks in the map
n = len(landmarks['id'])

# observedLandmarks is a vector that keeps track of which landmarks have been observed so far.
# observedLandmarks(i) will be true if the landmark with id = i has been observed at some point by the robot
observedLandmarks = scipy.zeros((n,)).astype(bool)#repmat(false,1,N);

# Initialize belief:
# mu: 2N+3x1 vector representing the mean of the normal distribution
# The first 3 components of mu correspond to the pose of the robot,
# and the landmark poses (xi, yi) are stacked in ascending id order.
# sigma: (2N+3)x(2N+3) covariance matrix of the normal distribution
mu = scipy.zeros((2*n + 3,))
robSigma = scipy.zeros((3, 3))
robMapSigma = scipy.zeros((3, 2*n))
mapSigma = infty*scipy.eye(2*n)
sigma = scipy.zeros((2*n + 3, 2*n + 3))
sigma[:3,:3] = robSigma
sigma[:3,3:] = robMapSigma
sigma[3:,:3] = robMapSigma.T
sigma[3:,3:] = mapSigma

# toogle the visualization type
showGui = True # show a window while the algorithm runs
#showGui = False # plot to files instead

# Perform filter update for each odometry-observation pair read from the
# data file.
for t in range(len(data['sensor'])):
#for t in range(50):

   # Perform the prediction step of the EKF
   mu, sigma = ekf_slam.prediction(mu, sigma, data['odometry'][t])

   # Perform the correction step of the EKF
   mu, sigma, observedLandmarks = ekf_slam.correction(mu, sigma, data['sensor'][t], observedLandmarks)

   #Generate visualization plots of the current state of the filter
   plot.plot_state(mu, sigma, landmarks, t, observedLandmarks, data['sensor'][t], showGui)
   print(r'Current state vector: \n mu = '),
   print(mu)

print("Final system covariance matrix: %f".format(sigma))
# Display the final state estimate
print("Final robot pose:")
print("mu_robot = ",  mu[:3],"sigma_robot = ", sigma[:3,:3])
