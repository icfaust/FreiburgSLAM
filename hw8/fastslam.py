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
    m = len(z['x'])
    
    # TODO: Construct the sensor noise matrix Q_t (2 x 2)
    
    # process each particle
    for i in xrange(numParticles): #particle loop
        robot = particles[i]['pose']
        # process each measurement
        for j in xrange(m): #measurement loop
            # Get the id of the landmark corresponding to the j-th observation
            # particles(i).landmarks(l) is the EKF for this landmark
            l = z['id'][j]

            # The (2x2) EKF of the landmark is given by
            # its mean particles(i).landmarks(l).mu
            # and by its covariance particles(i).landmarks(l).sigma

            # If the landmark is observed for the first time:
            if particles[i]['landmarks']['observed'] == False:

                # TODO: Initialize its position based on the measurement and the current robot pose:
            
                # get the Jacobian with respect to the landmark position
                [h, H] = main.measurement_model(particles[i], z[j])

                # TODO: initialize the EKF for this landmark

                # Indicate that this landmark has been observed
                particles[i]['landmarks']['observed'] = True
                
            else:

                # get the expected measurement
                [expectedZ, H] = main.measurement_model(particles[i], z[j])

                # TODO: compute the measurement covariance
                
                # TODO: calculate the Kalman gain
                
                # TODO: compute the error between the z and expectedZ (remember to normalize the angle)
                
                # TODO: update the mean and covariance of the EKF for this landmark
                
                # TODO: compute the likelihood of this observation, multiply with the former weight
                #       to account for observing several features in one time step


    return particles
