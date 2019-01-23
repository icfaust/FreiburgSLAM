#!/usr/bin/python
import scipy
import scipy.stats
import scipy.linalg
import maptlotlib.pyplot as plt
import main
import plot
import motion

# Read sensor readings, i.e. odometry
def motion():
    data = main.read_data('odometry.dat')
    
    noise = scipy.array([0.005, 0.01, 0.005])

    # how many particles
    numParticles = 100

    # initialize the particles array
    particles = [{'weight':1./numParticles,
                 'pose':scipy.zeros((3,)),
                 'history':[]} for x in range(numParticles)]

    
    #for i = 1:numParticles
    #  particles(i).weight = 1. / numParticles;
    #  particles(i).pose = zeros(3,1);
    #  particles(i).history = cell();
    #end

    # Perform filter update for each odometry-observation read from the
    # data file.
    for t in xrange(len(data['odometry'])):# = 1:size(data.timestep, 2)
        #for t = 1:50
        print('timestep = %d\n', t)

        # Perform the prediction step of the particle filter
        particles = motion.prediction(particles, data['odometry'][t], noise)

        # Generate visualization plots of the current state of the filter
        plot.plot_state(particles, t)

def resampling():
    # how many particles
    numParticles = 1000;

    # initialize the particles array

    particles = [{'weight':1./numParticles,
                  'pose':scipy.stats.norm.rvs([0., 0.], [1., 2.])
                  'history':[]} for in range(numParticles)]
    
    #p = {'weight':scipy.ones((numParticles,))/numParticles,
    #     'pose':scipy.stats.norm.rvs([0., 0.], [1., 2.]),
    #     'history':[[]]*numParticles}

    #for i = 1:numParticles
    #    particles(i).weight = 1. / numParticles;
    #    particles(i).pose = normrnd([0 0]', [1 2]');
    #    particles(i).history = cell();
    #end


    # re-weight the particles according to their distance to [0 0]
    sigma = .2*scipy.eye(2)
    sinv = scipy.linalg.inv(sigma)
    
    for p in particles:
        p['weight'] = scipy.exp(-.5*scipy.dot(p['pose'], scipy.dot(sinv, p['pose'])))
    #for i = 1:numParticles
    #particles(i).weight = exp(-1/2 * particles(i).pose' * inv(sigma) * particles(i).pose);
    #end

    resampledParticles = motion.resample(p)

    # plot the particles before (red) and after resampling (blue)
    plt.plot(p['pose'][:,0], p['pose'][:,1], 'r+', markersize=5.)
    plt.plot(resampledParticles['pose'][:,0], resampledParticles['pose'][:,1], 'b*', markersize=5.)
    plt.show()
