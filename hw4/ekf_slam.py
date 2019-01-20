import scipy
import scipy.linalg
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
    R[:3,:3] = R3

    # TODO: Compute the predicted sigma after incorporating the motion
    
    return mu, sigma


def correction(mu, sigma, z, observedLandmarks):
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
              Each observation z(i) has an id z[i]['id'], a range of
              z[i]['range'], and a bearing z[i]['bearing']. The 
              vector observed Landmarks indicates which landmarks 
              have been observed at some point by the robot.
           observedLandmarks (boolean numpy array): new landmark 
              signifier. False if the landmark with id = j has never
              been observed before.

       Returns:
           mu (numpy float array): updated mu
           sigma (numpy float array): updated sigma
           observedLandmarks (boolean numpy array): updated landmark
           signifier. 
    """


    # Number of measurements in this time step
    m = len(z)

    # Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
    # ExpectedZ: vectorized form of all expected measurements in the same form.
    # They are initialized here and should be filled out in the for loop below
    Z = scipy.zeros((2*m,))
    expectedZ = scipy.zeros((2*m,))

    # Iterate over the measurements and compute the H matrix
    # (stacked Jacobian blocks of the measurement function)
    # H will be 2m x 2N+3
    H = []

    for i in range(m):
	# Get the id of the landmark corresponding to the i-th observation
	landmarkId = z[i]['id']
	# If the landmark is obeserved for the first time:
	if observedLandmarks[landmarkId] == False:
	    # TODO: Initialize its pose in mu based on the measurement and the current robot pose:
		
	    # Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks[landmarkId] = True

	# TODO: Add the landmark measurement to the Z vector
	 
	# TODO: Use the current estimate of the landmark pose
	# to compute the corresponding expected measurement in expectedZ:

	# TODO: Compute the Jacobian Hi of the measurement function h for this observation
	
	# Augment H with the new Hi
	H += [Hi]	

    H = scipy.vstack(H)

    # TODO: Construct the sensor noise matrix Q

    # TODO: Compute the Kalman gain

    # TODO: Compute the difference between the expected and recorded measurements.
    # Remember to normalize the bearings after subtracting!
    # (hint: use the normalize_all_bearings function available in tools)

    # TODO: Finish the correction step by computing the new mu and sigma.
    # Normalize theta in the robot pose.

    return mu, sigma, observedLandmarks
