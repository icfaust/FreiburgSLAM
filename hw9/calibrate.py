import scipy
import scipy.linalg
import main


def apply_odometry_correction(X, U):
    """ computes a calibrated vector of odometry measurements
     by applying the bias term to each line of the measurements
     X: 	3x3 matrix obtained by the calibration process
     U: 	Nx3 matrix containing the odometry measurements
     C:	Nx3 matrix containing the corrected odometry measurements"""	
    
    # TODO: compute the calibrated motion vector, try to vectorize
    C = scipy.dot(X, U.T).T #this is sad and ugly
    return C


def compute_trajectory(U):
    """ computes the trajectory of the robot by chaining up
    the incremental movements of the odometry vector
    U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
    T:	a (N+1)x3 matrix, each row contains the robot position (starting from 0,0,0)"""
    
    # initialize the trajectory matrix
    T = scipy.zeros((len(U) + 1, 3))
    # store the first pose in the result
    T[0] = scipy.zeros((1,3))
    # the current pose in the chain
    currentPose = main.v2t(T[0])
    
    # TODO: compute the result of chaining up the odometry deltas
    # Note that U(i) results in T(i+1).
    # T(i+1) can be computed by calling t2v(currentPose)
    # after computing the current pose of the robot
    for i in range(len(U)):
        currentPose = scipy.dot(currentPose, main.v2t(U[i]))
        T[i+1] = main.t2v(currentPose)
        
    return T


def ls_calibrate_odometry(Z):
    """this function solves the odometry calibration problem
    given a measurement matrix Z.
    We assume that the information matrix is the identity
    for each of the measurements
    Every row of the matrix contains
    z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
    Z:	The measurement matrix
    X:	the calibration matrix
    returns the correction matrix X"""

    # initial solution (the identity transformation)

    X = scipy.eye(3) 
    
    # TODO: initialize H and b of the linear system
    b = scipy.zeros((X.size,1))
    H = scipy.zeros((X.size,X.size))
    # TODO: loop through the measurements and update H and b
    # You may call the functions _error_function and _jacobian, see below
    # We assume that the information matrix is the identity.
    for i in range(len(Z)):
        er = _error_function(i, X, Z)
        J = _jacobian(i, Z)
        b += scipy.dot(scipy.atleast_2d(er), J).T #This work assumes Omega is the identity matrix
        H += scipy.dot(J.T, J)

    print(H.shape,b.shape)
    X -= scipy.dot(scipy.linalg.inv(H), b).reshape((3,3))
    
    # TODO: solve and update the solution
    return X


def _error_function(i, X, Z):
    """ this function computes the error of the i^th measurement in Z
    given the calibration parameters
    i:	the number of the measurement
    X:	the actual calibration parameters
    Z:	the measurement matrix, each row contains first the scan-match result
       and then the motion reported by odometry
    out:the error of the ith measurement"""

    # TODO compute the error of each measurement
    out = Z[i,:3] - scipy.dot(X, Z[i,3:])

    return out


def _jacobian(i, Z):
    """ derivative of the error function for the ith measurement in Z
    i:	the measurement number
    Z:	the measurement matrix
    J:	the jacobian of the ith measurement"""

    J = scipy.zeros((3,9))
    # TODO compute the Jacobian
    J[0,:3] = -1*Z[i,3:]
    J[1,3:6] = -1*Z[i,3:]
    J[2,6:] = -1*Z[i,3:]

    return J
