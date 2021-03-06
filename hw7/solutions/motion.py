import scipy
import scipy.stats
import main

def prediction(particles, u, noise):
    """ Updates the particles by drawing from the motion model
    Use u['r1'], u['t'], and u['r2'] to access the rotation and translation values
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

    for p in particles:#= 1:numParticles
    
        # append the old position to the history of the particle
        p['history'] += [p['pose']]

        ur1 = scipy.stats.norm.rvs(scale=r1Noise) + u['r1']
        ut = scipy.stats.norm.rvs(scale=transNoise) + u['t']
        ur2 = scipy.stats.norm.rvs(scale=r2Noise) + u['r2']
    
        # TODO: sample a new pose for the particle
        p['pose'] += scipy.array([ut*scipy.cos(p['pose'][2] + ur1),
                                  ut*scipy.sin(p['pose'][2] + ur1),
                                  ur1 + ur2])
        p['pose'][2] = main.normalize_angle(p['pose'][2])      
    return particles


def resample(particles):
    """ resample the set of particles.
    A particle has a probability proportional to its weight to get
    selected. A good option for such a resampling method is the so-called low
    variance sampling, Probabilistic Robotics pg. 109"""
    numParticles = len(particles)
    w = scipy.array([p['weight'] for p in particles])

    # normalize the weight
    w = w / scipy.sum(w)

    # consider number of effective particles, to decide whether to resample or not
    useNeff = False
    #useNeff = true
    if useNeff:
        neff = 1. / scipy.sum(pow(w, 2))
        print(neff)
        if neff > 0.5*numParticles:
            newParticles = particles.copy()
            for i in xrange(numParticles):
                newParticles[i]['weight'] = w[i]
            return newParticles
    
    newParticles = [[]]*numParticles

    # TODO: implement the low variance re-sampling
    
    # the cumulative sum
    wsum = scipy.cumsum(w)

    # initialize the step and the current position on the roulette wheel
    spacing = wsum[-1]*(scipy.stats.uniform.rvs(scale = 1./numParticles) + scipy.mgrid[0.:1.:1./numParticles])
    idx = scipy.digitize(spacing, wsum)

    # walk along the wheel to select the particles
    for i in range(numParticles):
        newParticles[i] = particles[idx[i]]
    
    return newParticles
