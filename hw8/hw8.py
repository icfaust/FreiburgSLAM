#!/usr/bin/python
import scipy
import fastslam
import main

# This is the main FastSLAM loop. This script calls all the required
# functions in the correct order.
#
# You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should however not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions you
# should read their documentation which tells you the expected dimensions.
# Read world data, i.e. landmarks. The true landmark positions are not given
# to the robot

landmarks = main.read_world('../world.dat')
# Read sensor readings, i.e. odometry and range-bearing sensor
data = main.read_data('../sensor_data.dat')

# Get the number of landmarks in the map
n = len(landmarks['id'])

noise = scipy.array([0.005, 0.01, 0.005])

# how many particles
numParticles = 100

# THIS IS VERY MATLABED I NEED TO REDO THIS AGAIN
# initialize the particles dict
particles = [{'weight':1./numParticles,
              'pose':scipy.zeros((3,)),
              'history':[],
              'landmarks':[{'observed':False,
                            'mu':scipy.zeros((2,1)),
                            'sigma':scipy.zeros((2,2))} for i in n]} for x in range(numParticles)]

# toogle the visualization type
showGui = True  # show a window while the algorithm runs
#showGui = False # plot to files instead

# Perform filter update for each odometry-observation pair read from the
# data file.
for t in range(len(data['odometry'])):
#for t in range(50):
    print('timestep = %d\n' % t)

    # Perform the prediction step of the particle filter
    particles = main.prediction(particles, data['odometry'][t], noise)

    # Perform the correction step of the particle filter
    particles = fastslam.correction(particles, data['sensor'][t])

    # Generate visualization plots of the current state of the filter
    plot.plot_state(particles, landmarks, t, data['sensor'][t], showGui)

    # Resample the particle set
    particles = main.resample(particles)

