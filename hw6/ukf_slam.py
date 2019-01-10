import scipy
import main

def prediction(mu, sigma, u):
    """Updates the belief concerning the robot pose according to 
       the motion model (From the original MATLAB code: Use u.r1,
       u.t, and u.r2 to access the rotation and translation values)
       In this case u['r1'], u['t'] and u['r2'] to access values

       Args:
           mu ((2N+3, 1) numpy float array): state mean matrix
               mean. N in this case is the number of landmarks
           sigma ((2N+3, 2N+3) numpy float array): covariance matrix
           u (dictionary): odometry reading (r1, t, r2)

       Returns:
           mu (numpy float array): updated mu by u
           sigma (numpy float array): updated sigma by u
       """

    # TODO: Compute the new mu based on the noise-free (odometry-based) motion model
    # Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)

    # TODO: Compute the 3x3 Jacobian Gx of the motion model


    # TODO: Construct the full Jacobian G


    # Motion noise
    motionNoise = 0.1
    R3 = scipy.array([[motionNoise, 0., 0.], 
                      [0., motionNoise, 0.], 
                      [0., 0., motionNoise/10.]])
    
    R = scipy.zeros((sigma.shape[0],sigma.shape[0]))
    R[0:3,0:3] = R3

    # TODO: Compute the predicted sigma after incorporating the motion
    
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
    #global scale;

    # Number of measurements in this time step
    m = z.shape[1]

    # Measurement noise
    Q = 0.01*scipy.eye(2)
    i = 0
    lm = len(m)
    
    while i < lm:#1:m

	# If the landmark is observed for the first time:
	if (scipy.sum(mapout == z[i].id) == 0):
            # Add new landmark to the map
            [mu, sigma, mapout] = main.add_landmark_to_map(mu,
                                                           sigma,
                                                           z[i],
                                                           mapout,
                                                           Q,
                                                           scale)
	    # The measurement has been incorporated so we quit the correction step
            i = lm
        else:
	    # Compute sigma points from the predicted mean and covariance
            # This corresponds to line 6 on slide 32
	    sigma_points = main.compute_sigma_points(mu, sigma, scale)
            # Normalize!
	    sigma_points[2,:] = main.normalize_angle(sigma_points[2,:])
            
	    # Compute lambda
	    n = len(mu)
	    num_sig = sigma_points.shape[1]
	    lam = scale - n
            
            # extract the current location of the landmark for each sigma point
            # Use this for computing an expected measurement, i.e., applying the h function
	    landmarkIndex = mapout == z[i].id
	    landmarkXs = sigma_points[2*landmarkIndex + 1, :]
	    landmarkYs = sigma_points[2*landmarkIndex + 2, :]
            
	    # TODO: Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
            # This corresponds to line 7 on slide 32
                        
            # setup the weight vector for mean and covariance
            wm = [lam/scale, scipy.tile(1/(2*scale), (1, 2*n))]

	    # TODO: Compute zm, line 8 on slide 32
	    # zm is the recovered expected measurement mean from z_points.
	    # It will be a 2x1 vector [expected_range; expected_bearing].
            # For computing the expected_bearing compute a weighted average by
            # summing the sines/cosines of the angle
            
            
	    # TODO: Compute the innovation covariance matrix S (2x2), line 9 on slide 32
            # Remember to normalize the bearing after computing the difference
            
            
	    # TODO: Compute Sigma_x_z, line 10 on slide 32
            # (which is equivalent to sigma times the Jacobian H transposed in EKF).
	    # sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
            # Remember to normalize the bearing after computing the difference
            
            
	    # TODO: Compute the Kalman gain, line 11 on slide 32
            
            
	    # Get the actual measurement as a vector (for computing the difference to the observation)
	    z_actual = [z[i].range, z[i].bearing]
            
	    # TODO: Update mu and sigma, line 12 + 13 on slide 32
            # normalize the relative bearing
            
	    # TODO: Normalize the robot heading mu(3)

            # Don't touch this iterator
            i = i + 1
            
    return mu, sigma, observedLandmarks
