import scipy
import scipy.stats
import scipy.linalg
import main

def correction(particles, z):
    """ Weight the particles according to the current map of the particle
    and the landmark observations z.
    z: dict containing the landmark observations.
    Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
    The vector observedLandmarks indicates which landmarks have been observed
    at some point by the robot."""
    
    # Number of particles
    numParticles = len(particles)
    
    # Number of measurements in this time step
    m = len(z)
    
    # TODO: Construct the sensor noise matrix Q_t (2 x 2)
    Q_t = .1*scipy.eye(2)
    
    # process each particle
    for p in particles: #range(numParticles): #particle loop
        robot = p['pose']
        
        # process each measurement
        for j in range(m): #measurement loop
            # Get the id of the landmark corresponding to the j-th observation
            # particles[i]['landmarks'][l] is the EKF for this landmark
            l = z[j]['id']

            # The (2x2) EKF of the landmark is given by
            # its mean particles[i]['landmarks'][l]['mu']
            # and by its covariance particles[i]['landmarks'][l]['sigma']
            # If the landmark is observed for the first time:
            if not p['landmarks'][l]['observed']:

                # TODO: Initialize its position based on the measurement and the current robot pose:
                r = z[j]['range']
                t = z[j]['bearing']
                p['landmarks'][l]['mu'][0] = robot[0] + r*scipy.cos(robot[2] + t)
                p['landmarks'][l]['mu'][1] = robot[1] + r*scipy.sin(robot[2] + t)
                
                # get the Jacobian with respect to the landmark position
                h, H = main.measurement_model(p, z[j])
                Hinv = scipy.linalg.inv(H)

                # TODO: initialize the EKF for this landmark
                p['landmarks'][l]['sigma'] = scipy.dot(Hinv, scipy.dot(Q_t, Hinv.T))
                
                # Indicate that this landmark has been observed
                p['landmarks'][l]['observed'] = True
                
            else:

                # get the expected measurement
                expectedZ, H = main.measurement_model(p, z[j])
                # TODO: compute the measurement covariance
                Qinv = scipy.linalg.inv(scipy.dot(H, scipy.dot(p['landmarks'][l]['sigma'], H.T)) + Q_t)
                
                # TODO: calculate the Kalman gain
                K = scipy.dot(p['landmarks'][l]['sigma'], scipy.dot(H.T, Qinv))
                
                # TODO: compute the error between the z and expectedZ (remember to normalize the angle)
                delta = scipy.array([z[j]['range'], z[j]['bearing']]) - expectedZ
                delta[1] = main.normalize_angle(delta[1])
                
                # TODO: update the mean and covariance of the EKF for this landmark
                p['landmarks'][l]['mu'] += scipy.dot(K, delta)
                p['landmarks'][l]['sigma'] = scipy.dot((scipy.eye(2) - scipy.dot(K, H)), p['landmarks'][l]['sigma'])
                
                # TODO: compute the likelihood of this observation, multiply with the former weight
                #       to account for observing several features in one time step

                det = pow(scipy.linalg.det(Qinv/(2*scipy.pi)), .5)
                p['weight'] *= det*scipy.exp(-.5*scipy.dot(delta.T, scipy.dot(Qinv, delta)))
                
    return particles
