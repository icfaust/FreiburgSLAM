import scipy
import main


def apply_odometry_correction(X, U):
    """ computes a calibrated vector of odometry measurements
     by applying the bias term to each line of the measurements
     X: 	3x3 matrix obtained by the calibration process
     U: 	Nx3 matrix containing the odometry measurements
     C:	Nx3 matrix containing the corrected odometry measurements"""	
    
    # TODO: compute the calibrated motion vector, try to vectorize
    C = []
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
    T = []
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
    
    # TODO: loop through the measurements and update H and b
    # You may call the functions _error_function and _jacobian, see below
    # We assume that the information matrix is the identity.
    
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

    out = [] 

    return out


def _jacobian(i, Z):
    """ derivative of the error function for the ith measurement in Z
    i:	the measurement number
    Z:	the measurement matrix
    J:	the jacobian of the ith measurement"""

    J = []
    # TODO compute the Jacobian

    return J
