import scipy
import hw8
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
N = len(landmarks,2);

noise = scipy.array([0.005, 0.01, 0.005])

# how many particles
numParticles = 100

# THIS IS VERY MATLABED I NEED TO REDO THIS AGAIN
# initialize the particles dict
particles = {'weight':scipy.zeros((numParticles,)),
             'pose':scipy.zeros((numParticles, 3)),
             'history':scipy.zeros((numParticles, 3))}
for i in xrange(numParticles):# = 1:numParticles
    particles['weight'][i] = 1. / numParticles;
    #particles(i).pose = zeros(3, 1);
    #particles(i).history = cell();
    for l in xrange(N):# = 1:N % initialize the landmarks aka the map
        particles(i).landmarks(l).observed = False
        #% 2D position of the landmark
        particles(i).landmarks(l).mu = scipy.zeros(2,1)
        #covariance of the landmark
        particles(i).landmarks(l).sigma = scipy.zeros(2,2)



# toogle the visualization type
#showGui = True;  % show a window while the algorithm runs
showGui = False % plot to files instead

# Perform filter update for each odometry-observation pair read from the
# data file.
for t in xrange(data.timestep.shape[1]):# 1:size(data.timestep, 2)
#for t = 1:50
    print('timestep = %d\n', t)

    # Perform the prediction step of the particle filter
    particles = hw8.prediction_step(particles, data.timestep[t].odometry, noise)

    # Perform the correction step of the particle filter
    particles = hw8.correction_step(particles, data.timestep[t].sensor)

    # Generate visualization plots of the current state of the filter
    plot_state(particles, landmarks, t, data.timestep(t).sensor, showGui)

    # Resample the particle set
    particles = resample(particles)

