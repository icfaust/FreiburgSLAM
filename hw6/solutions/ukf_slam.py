import scipy
import main

def prediction(mu, sigma, u, scale):
    """Updates the belief concerning the robot pose according to the motion model.
    mu: state vector containing robot pose and poses of landmarks obeserved so far
    Current robot pose = mu(1:3)
    Note that the landmark poses in mu are stacked in the order by which they were observed
    sigma: the covariance matrix of the system.
    u: odometry reading (r1, t, r2)
    Use u['r1'], u['t'], and u['r2'] to access the rotation and translation values"""

    # For computing lambda.
    # use the scale parameter

    # Compute sigma points
    sigma_points = main.compute_sigma_points(mu, sigma, scale)

    # Dimensionality
    n = len(mu)
    # lambda
    lamb = scale - n

    # TODO: Transform all sigma points according to the odometry command
    # Remember to vectorize your operations and normalize angles
    # Tip: the function normalize_angle also works on a vector (row) of angles
    sigma_points[0] += u['t']*scipy.cos(sigma_points[2] + u['r1'])
    sigma_points[1] += u['t']*scipy.sin(sigma_points[2] + u['r1']) #the inclusion of different sigma_points[2] causes it to be nonlinear
    sigma_points[2] = main.normalize_angle(sigma_points[2] + u['r1'] + u['r2'])
    
    # Computing the weights for recovering the mean
    wm = scipy.concatenate([[lamb/scale], scipy.ones((2*n,))/(2*scale)])
    wc = wm.copy()

    # TODO: recover mu.
    # Be careful when computing the robot's orientation (sum up the sines and
    # cosines and recover the 'average' angle via atan2)
    cosines = scipy.sum(scipy.cos(sigma_points[2])*wm)
    sines = scipy.sum(scipy.sin(sigma_points[2])*wm)
    
    # recompute the angle and normalize it
    mu_theta = scipy.arctan2(sines, cosines)
    mu = scipy.sum(sigma_points*scipy.tile(wm, (sigma_points.shape[0], 1)), 1)
    mu[2] = mu_theta
    
    # TODO: Recover sigma. Again, normalize the angular difference
    diff = sigma_points - scipy.tile(mu, (sigma_points.shape[1],1)).T

    # Normalize!
    diff[2,:] = main.normalize_angle(diff[2])
    sigma = scipy.dot(scipy.tile(wc, (diff.shape[0], 1))*diff, diff.T)

    # Motion noise
    motionNoise = 0.1
    R3 = scipy.array([[motionNoise, 0., 0.], 
                      [0., motionNoise, 0.], 
                      [0., 0., motionNoise/10.]])
    
    #R = scipy.zeros((sigma.shape[0],sigma.shape[0]))
    #R[:3,:3] = R3

    # TODO: Compute the predicted sigma after incorporating the motion
    sigma[:3,:3] += R3
    
    return mu, sigma


def correction(mu, sigma, z, mapout, scale):
    """Updates the belief, i. e., mu and sigma after observing
       landmarks, according to the sensor model. The employed sensor
       model measures the range and bearing of a landmark.

       Args:
           mu ((2N+3, 1) numpy float array): state mean matrix
              The first 3 components of mu correspond to the current
              estimate of the robot pose [x, y, theta] The current
              pose estimate of the landmark with id = j is:
              [mu[2*j+2], mu[2*j+3]]
           sigma ((2N+3, 2N+3) numpy float array): covariance matrix
           z: landmark observations.
              Each observation z(i) has an id z(i).id, a range z(i).
              range, and a bearing z(i).bearing. The vector observed
              Landmarks indicates which landmarks have been observed
              at some point by the robot.
           observedLandmarks (boolean numpy array): new landmark 
              signifier. False if the landmark with id = j has never
              been observed before.

       Returns:
           mu (numpy float array): updated mu
           sigma (numpy float array): updated sigma
           observedLandmarks (boolean numpy array): updated landmark            
           signifier. 
    """

    # For computing sigma

    # Number of measurements in this time step
    m = len(z)

    # Measurement noise
    Q = 0.01*scipy.eye(2)
    
    for i in range(m):#1:m
	# If the landmark is observed for the first time:
	if ~scipy.any(mapout == z[i]['id']):
            # Add new landmark to the map
            [mu, sigma, mapout] = main.add_landmark_to_map(mu,
                                                           sigma,
                                                           z[i],
                                                           mapout,
                                                           Q,
                                                           scale)
	    # The measurement has been incorporated so we quit the correction step
        else:
	    # Compute sigma points from the predicted mean and covariance
            # This corresponds to line 6 on slide 32
	    sigma_points = main.compute_sigma_points(mu, sigma, scale)
            # Normalize!
	    sigma_points[2] = main.normalize_angle(sigma_points[2])
	    # Compute lambda
	    n = len(mu)
	    num_sig = sigma_points.shape[1]
	    lam = scale - n
            
            # extract the current location of the landmark for each sigma point
            # Use this for computing an expected measurement, i.e., applying the h function
	    landmarkIndex = scipy.where(mapout == z[i]['id'])[0]
            landmarks = scipy.zeros((2, sigma_points.shape[1]))
	    landmarks[0] = sigma_points[2*landmarkIndex + 3]
	    landmarks[1] = sigma_points[2*landmarkIndex + 4]
            
	    # TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
            # This corresponds to line 7 on slide 32
            z_points = scipy.zeros((2, 2*n + 1))
            z_points[0] = scipy.sqrt(pow(landmarks[0] - sigma_points[0], 2) + pow(landmarks[1] - sigma_points[1], 2))
            z_points[1] = main.normalize_angle(scipy.arctan2(landmarks[1] - sigma_points[1],
                                                             landmarks[0] - sigma_points[0]) - sigma_points[2])

            # setup the weight vector for mean and covariance 
            wm = scipy.concatenate([[lam/scale], scipy.tile(1/(2*scale), (2*n,))])
            wc = wm.copy()
            
	    # TODO: Compute zm, line 8 on slide 32
	    # zm is the recovered expected measurement mean from z_points.
	    # It will be a 2x1 vector [expected_range; expected_bearing].
            # For computing the expected_bearing compute a weighted average by
            # summing the sines/cosines of the angle
            zm = scipy.zeros((2,))
            zm[0] = scipy.sum(wm*z_points[0])
            zm[1] = scipy.arctan2(scipy.sum(wm*scipy.sin(z_points[1])),
                                  scipy.sum(wm*scipy.cos(z_points[1])))
            
	    # TODO: Compute the innovation covariance matrix S (2x2), line 9 on slide 32
            # Remember to normalize the bearing after computing the difference
            z_points[0] -= zm[0] #no longer necessary to distinguish Zeta from Zeta-z_hat
            z_points[1] = main.normalize_angle(z_points[1] - zm[1])
            S = scipy.dot(z_points, (scipy.tile(wc, (2,1))*z_points).T) + Q

	    # TODO: Compute Sigma_x_z, line 10 on slide 32
            # (which is equivalent to sigma times the Jacobian H transposed in EKF).
	    # sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
            # Remember to normalize the bearing after computing the difference
            temp = sigma_points - scipy.tile(sigma_points[:,0], (num_sig, 1)).T
            temp[2] = main.normalize_angle(temp[2])
            sigma_x_z =  scipy.dot(temp,
                                   (scipy.tile(wc, (2,1))*z_points).T)
            
	    # TODO: Compute the Kalman gain, line 11 on slide 32
            K = scipy.dot(sigma_x_z, scipy.linalg.inv(S))
            
	    # Get the actual measurement as a vector (for computing the difference to the observation)
	    z_actual = scipy.array([z[i]['range'], z[i]['bearing']])
            
	    # TODO: Update mu and sigma, line 12 + 13 on slide 32
            # normalize the relative bearing
            zin = z_actual - zm
            zin[1] = main.normalize_angle(zin[1])
            mu += scipy.dot(K, zin)

            sigma -= scipy.dot(K, scipy.dot(S, K.T)) 
	    # TODO: Normalize the robot heading mu[2]
            mu[2] = main.normalize_angle(mu[2])
            
    return mu, sigma, mapout
