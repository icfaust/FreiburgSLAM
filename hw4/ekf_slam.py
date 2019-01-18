import scipy
import scipy.linalg
import main


def prediction(mu, sigma, u):
    """Updates the belief concerning the robot pose according to 
       the motion model (From the original MATLAB code: Use u.r1,
       u.t, and u.r2 to access the rotation and translation values)
       In this case u['r1'], u['t'] and u['r2'] to access values.

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

    mu[:3] += scipy.array([u['t']*scipy.cos(u['t'] + u['r1']),
                           u['t']*scipy.sin(u['t'] + u['r1']),
                           u['r1'] + u['r2']])
    mu[2] = main.normalize_angle(mu[2])
    
    # TODO: Compute the 3x3 Jacobian Gx of the motion model
    Gx = scipy.eye(3)
    Gx[0,2] = -1*u['t']*scipy.sin(mu[2] + u['r1'])
    Gx[1,2] = u['t']*scipy.cos(mu[2] + u['r2'])
    
    # TODO: Construct the full Jacobian G


    # Motion noise
    motionNoise = 0.1
    R3 = scipy.array([[motionNoise, 0., 0.], 
                      [0., motionNoise, 0.], 
                      [0., 0., motionNoise/10.]])
    
    #R = scipy.zeros((sigma.shape[0],sigma.shape[0]))
    #R[:3,:3] = R3

    # TODO: Compute the predicted sigma after incorporating the motion

    #handling the 3x3 [x,y,theta] covariances
    sigma[:3,:3] = scipy.dot(Gx.T, scipy.dot(sigma[:3,:3], Gx)) + R3
    temp = scipy.dot(sigma[3:,:3], Gx)

    #handling the landmark/ [x,y,theta] covariances
    sigma[:3,3:] = temp.T
    sigma[3:,:3] = temp

    #I did it this way, as it was recommended in the lectures, it
    #reduces unneccesary computations elsewhere in what could be a
    #very large matrix (in generating a full Jacobian G)
        
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
        print(landmarkId)
	# If the landmark is obeserved for the first time:
	if observedLandmarks[landmarkId] == False:
	    # TODO: Initialize its pose in mu based on the measurement and the current robot pose:
            mu[2*landmarkId + 3] = mu[0] + z[i]['range']*scipy.cos(z[i]['bearing'] + mu[2]) #x position of landmark
            mu[2*landmarkId + 4] = mu[1] + z[i]['range']*scipy.sin(z[i]['bearing'] + mu[2]) #y position of landmark
                                                                                     
	    # Indicate in the observedLandmarks vector that this landmark has been observed
	    observedLandmarks[landmarkId] = True

	# TODO: Add the landmark measurement to the Z vector
	Z[2*i] = z[i]['range']
        Z[2*i + 1] = main.normalize_angle(z[i]['bearing']) #this is really inefficient code brought on by the hw designer

        
	# TODO: Use the current estimate of the landmark pose
	# to compute the corresponding expected measurement in expectedZ:

        expectedZ[2*i] = scipy.sqrt(pow(mu[0] - mu[2*landmarkId + 3], 2) + pow(mu[1] - mu[2*landmarkId + 4], 2)) #again, this could have been vectorized with a different approach to the landmarks
        expectedZ[2*i + 1] = main.normalize_angle(scipy.arctan2(mu[2*landmarkId + 4] - mu[1], mu[2*landmarkId + 3] - mu[0]) - mu[2])
        
	# TODO: Compute the Jacobian Hi of the measurement function h for this observation
        Hi = scipy.zeros((2, len(mu)))
        ux = mu[2*landmarkId + 3] - mu[0]# - z[i]['range']*scipy.cos(z[i]['bearing'] + mu[2])
        uy = mu[2*landmarkId + 4] - mu[1]# - z[i]['range']*scipy.sin(z[i]['bearing'] + mu[2])
        r = scipy.sqrt(pow(ux,2) + pow(uy,2))

        Hi[:,0] = scipy.array([-ux*r, uy])
        Hi[:,1] = scipy.array([-uy*r, -ux])
        Hi[:,2] = scipy.array([0., -r])
        Hi[:,2*landmarkId + 3] = scipy.array([ux*r, -uy])
	Hi[:,2*landmarkId + 4] = scipy.array([uy*r, ux])
	# Augment H with the new Hi
	H += [Hi/pow(r,2)]	

        
    H = scipy.vstack(H)
    print(Z,expectedZ)
    
    #TODO: Construct the sensor noise matrix Q
    Q = .01*scipy.eye(2*m)
    
    # TODO: Compute the Kalman gain
    inv = scipy.linalg.inv(scipy.dot(H, scipy.dot(sigma,
                                                  H.T)) + Q)
    K = scipy.dot(sigma, scipy.dot(H.T,inv))
    # TODO: Compute the difference between the expected and recorded measurements.
    # Remember to normalize the bearings after subtracting!
    # (hint: use the normalize_all_bearings function available in tools)
    delta = main.normalize_all_bearings(Z - expectedZ)

    # TODO: Finish the correction step by computing the new mu and sigma.
    # Normalize theta in the robot pose.
    mu += scipy.dot(K, delta)
    sigma += scipy.dot(scipy.eye(len(sigma)) - scipy.dot(K, H),
                       sigma)

    return mu, sigma, observedLandmarks
