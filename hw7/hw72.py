#!/usr/bin/python
import scipy
import scipy.stats
import scipy.linalg
import matplotlib.pyplot as plt
import main
import plot
import motion

# Read sensor readings, i.e. odometry
def motiontest():
    data = main.read_data('odometry.dat')
    
    noise = scipy.array([0.005, 0.01, 0.005])

    # how many particles
    numParticles = 100

    # initialize the particles array
    particles = [{'weight':1./numParticles,
                 'pose':scipy.zeros((3,)),
                 'history':[]} for x in range(numParticles)]

    # Perform filter update for each odometry-observation read from the
    # data file.
    for t in range(len(data['odometry'])):
        #for t in range(50):
        print('timestep = %d' % t)

        # Perform the prediction step of the particle filter
        particles = motion.prediction(particles, data['odometry'][t], noise)

        # Generate visualization plots of the current state of the filter
        plot.plot_state(particles, t)

def resamplingtest():
    # how many particles
    numParticles = 1000

    # initialize the particles array

    particles = [{'weight':1./numParticles,
                  'pose':scipy.stats.norm.rvs([0., 0.], [1., 2.]),
                  'history':[]} for x in range(numParticles)]

    # re-weight the particles according to their distance to [0 0]
    sigma = .2*scipy.eye(2)
    sinv = scipy.linalg.inv(sigma)
    
    for p in particles:
        p['weight'] = scipy.exp(-.5*scipy.dot(p['pose'], scipy.dot(sinv, p['pose'])))

    resampledParticles = motion.resample(particles)

    # plot the particles before (red) and after resampling (blue)
    parts = scipy.vstack([p['pose'] for p in particles])
    plt.plot(parts[:,0], parts[:,1], 'r+', markersize=5.)

    resamp = scipy.vstack([p['pose'] for p in resampledParticles])
    plt.plot(resamp[:,0], resamp[:,1], 'b*', markersize=5.)
    plt.show()
