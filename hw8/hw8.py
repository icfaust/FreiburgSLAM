import scipy
import scipy.stats
import main

def correction_step(particles, z):
    """ Weight the particles according to the current map of the particle
    and the landmark observations z.
    z: struct array containing the landmark observations.
    Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
    The vector observedLandmarks indicates which landmarks have been observed
    at some point by the robot."""
    
    # Number of particles
    numParticles = len(particles)
    
    # Number of measurements in this time step
    m = z.shape[1]
    
    # TODO: Construct the sensor noise matrix Q_t (2 x 2)
    
    # process each particle
    for i in xrange(numParticles): #particle loop
        robot = particles(i).pose;
        # process each measurement
        for j in xrange(m): #measurement loop
            # Get the id of the landmark corresponding to the j-th observation
            # particles(i).landmarks(l) is the EKF for this landmark
            l = z[j].id;

            # The (2x2) EKF of the landmark is given by
            # its mean particles(i).landmarks(l).mu
            # and by its covariance particles(i).landmarks(l).sigma

            # If the landmark is observed for the first time:
            if particles[i].landmarks(l).observed == False:

                # TODO: Initialize its position based on the measurement and the current robot pose:
            
                # get the Jacobian with respect to the landmark position
                [h, H] = main.measurement_model(particles(i), z(j));

                # TODO: initialize the EKF for this landmark

                # Indicate that this landmark has been observed
                particles(i).landmarks(l).observed = true;
                
            else:

                # get the expected measurement
                [expectedZ, H] = measurement_model(particles(i), z(j));

                # TODO: compute the measurement covariance
                
                # TODO: calculate the Kalman gain
                
                # TODO: compute the error between the z and expectedZ (remember to normalize the angle)
                
                # TODO: update the mean and covariance of the EKF for this landmark
                
                # TODO: compute the likelihood of this observation, multiply with the former weight
                #       to account for observing several features in one time step


    return particles


def resample(particles):
    """ resample the set of particles.
    A particle has a probability proportional to its weight to get
    selected. A good option for such a resampling method is the so-called low
    variance sampling, Probabilistic Robotics pg. 109"""
    
    numParticles = len(particles)
    
    w = particles.weight

    # normalize the weight
    w = w / scipy.sum(w)

    # consider number of effective particles, to decide whether to resample or not
    useNeff = False
    #useNeff = True
    if useNeff:
        neff = 1. / sum(w.^2);
        if neff > 0.5*numParticles:
            newParticles = particles
            for i in xrange(numParticles):
                newParticles[i].weight = w[i]
            return;


    #newParticles = struct;
    
    # TODO: implement the low variance re-sampling

    # the cummulative sum
    cs = scipy.cumsum(w)
    weightSum = cs[(len(cs)]

    # initialize the step and the current position on the roulette wheel
    step = weightSum / numParticles
    position = scipy.stats.uniform.rvs(0, scale=weightSum)
    idx = 0

    # walk along the wheel to select the particles
    for i in xrange(numParticles):# 1:numParticles
        position += step;
        if position > weightSum: #Is this necessary???
            position -= weightSum; #I have a feeling this was
                   #dubiously programmed...
            idx = 0
        while position > cs[idx]:
            idx++

        newParticles[i] = particles[idx]
        newParticles[i].weight = 1/numParticles

return newParticles

