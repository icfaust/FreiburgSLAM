#!/usr/bin/python
import scipy
import main
import plot
import ukf_slam

# This is the main unscented Kalman filter SLAM loop. This script calls all the required
# functions in the correct order.
#
# You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should however not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions you
# should read their documentation which tells you the expected dimensions.

# Turn off pagination:
#close all
#clear all
#more off;

#format long

# Make tools available
#addpath('tools');

# Read world data, i.e. landmarks. The true landmark positions are not given to the robot
landmarks = main.read_world('../world.dat')
#load landmarks
# Read sensor readings, i.e. odometry and range-bearing sensor
data = main.read_data('../sensor_data.dat')
# load data
# Initialize belief
mu = scipy.zeros(3,1)
sigma = 0.001*scipy.eye(3)
mapout = []

# For computing lambda
# scale = lam + dimensionality
#global scale; I hate globals, so I have set it as a parameter to the underlying functions
scale = 3.0

# toogle the visualization type
showGui = True;  # show a window while the algorithm runs
#showGui = False # plot to files instead

# Perform filter update for each odometry-observation pair read from the
# data file.
for t in range(len(data['odometry'])):#1:data.timestep.shape[1]:
    print('Time step t = %f'.format(t))

    # Perform the prediction step of the UKF
    mu, sigma = ukf_slam.prediction_step(mu, sigma, data['odometry'][t], scale)

    # Perform the correction step of the UKF
    mu, sigma, mapout = ukf_slam.correction_step(mu, sigma, data['sensor'][t], mapout, scale)

    #Generate visualization plots of the current state of the filter
    main.plot_state(mu, sigma, landmarks, t, mapout, data['sensor'][t], showGui)

    print("Current state vector mu ="),
    print(mu)
    print("Map contains the following landmarks:"),
    print(mapout)


#disp("Final system covariance matrix:"), disp(sigma)
# Display the final state estimate
print("Final robot pose:")
print("mu_robot = "),
print(mu[:2]),
print("sigma_robot = "),
print(sigma[:2,:2])

