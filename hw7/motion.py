import scipy

def prediction_step(particles, u, noise):
    """ Updates the particles by drawing from the motion model
    Use u.r1, u.t, and u.r2 to access the rotation and translation values
    which have to be pertubated with Gaussian noise.
    The position of the i-th particle is given by the 3D vector
    particles(i).pose which represents (x, y, theta).
    
    noise parameters
    Assume Gaussian noise in each of the three parameters of the motion model.
    These three parameters may be used as standard deviations for sampling."""

    r1Noise = noise[0]
    transNoise = noise[1]
    r2Noise = noise[2]

    numParticles = len(particles)

    for i in xrange(numParticles):#= 1:numParticles
    
        # append the old position to the history of the particle
        particles['history'][i] += [particles['pose'][i]]
    
        # TODO: sample a new pose for the particle
        
    return particles


def resample(particles):
    """ resample the set of particles.
    A particle has a probability proportional to its weight to get
    selected. A good option for such a resampling method is the so-called low
    variance sampling, Probabilistic Robotics pg. 109"""
    numParticles = len(particles)

    w = particles['weight']

    # normalize the weight
    w = w / scipy.sum(w)

    # consider number of effective particles, to decide whether to resample or not
    useNeff = False
    #useNeff = true
    if useNeff:
        neff = 1. / scipy.sum(pow(w, 2))
        print(neff)
        if neff > 0.5*numParticles:
            newParticles = particles
            newParticles['weight'] = w
            return newParticles
    
    newParticles = {'weight':[],
                    'pose':[],
                    'history':[]}#struct;

    # TODO: implement the low variance re-sampling
    
    # the cumulative sum
    
    # initialize the step and the current position on the roulette wheel
    
    # walk along the wheel to select the particles
    for i in xrange(numParticles):#= 1:numParticles
        pass
        
    return newParticles
