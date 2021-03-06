#!/usr/bin/python
import scipy
import scipy.linalg
import lsslam
from lsslam import _lsSLAM as lsSLAM

    
def test_jacobian_pose_pose():

    eps = 1e-5

    x1 = scipy.array([1.1, 0.9, 1])
    x2 = scipy.array([2.2, 1.85, 1.2])
    z  = scipy.array([0.9, 1.1, 1.05])

    # get the analytic Jacobian
    e, A, B = lsslam.linearize_pose_pose_constraint(x1, x2, z)

    # check the error vector
    e_true = scipy.array([-1.06617,  -1.18076,  -0.85000])
    if scipy.linalg.norm(e - e_true) > eps:
        print('Your error function seems to return a wrong value')
        print('Result of your function', e)
        print('True value', e_true)
    else:
        print('The computation of the error vector appears to be correct')

    # compute it numerically
    delta = 1e-6
    scalar = 1. / (2*delta)

    # test for x1
    ANumeric = scipy.zeros((3,3))
    for d in range(3):
        curX = x1.copy()
        curX[d] += delta
        err = lsslam.linearize_pose_pose_constraint(curX, x2, z)[0]
        temp1 = lsslam.linearize_pose_pose_constraint(curX, x2, z)[0]
        curX[d] -= 2*delta
        err -= lsslam.linearize_pose_pose_constraint(curX, x2, z)[0]
        temp2 = lsslam.linearize_pose_pose_constraint(curX, x2, z)[0]
        
        ANumeric[:, d] = scalar * err


    diff = ANumeric - A
    if (abs(diff)).max() > eps:
        print('Error in the Jacobian for x1')
        print('Your analytic Jacobian', A)
        print('Numerically computed Jacobian', ANumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x1 appears to be correct')

    #test for x2
    BNumeric = scipy.zeros((3,3))
    for d in range(3):
        curX = x2.copy()
        curX[d] += delta
        err = lsslam.linearize_pose_pose_constraint(x1, curX, z)[0]
        curX[d] -= 2*delta
        err -= lsslam.linearize_pose_pose_constraint(x1, curX, z)[0]

        BNumeric[:, d] = scalar * err


    diff = BNumeric - B
    if (abs(diff)).max() > eps:
        print('Error in the Jacobian for x2')
        print('Your analytic Jacobian', B)
        print('Numerically computed Jacobian', BNumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x2 appears to be correct')


def test_jacobian_pose_landmark():
    eps = 1e-5

    x1 = scipy.array([1.1, 0.9, 1.])
    x2 = scipy.array([2.2, 1.9])
    z  = scipy.array([1.3, -0.4])

    # get the analytic Jacobian
    e, A, B = lsslam.linearize_pose_landmark_constraint(x1, x2, z)

    # check the error vector
    e_true = scipy.array([0.135804, 0.014684])
    if scipy.linalg.norm(e - e_true) > eps:
        print('Your error function seems to return a wrong value')
        print('Result of your function', e)
        print('True value', e_true)
    else:
        print('The computation of the error vector appears to be correct')

    # compute it numerically
    delta = 1e-6
    scalar = 1. / (2*delta)

    # test for x1
    ANumeric = scipy.zeros((2,3))
    for d in range(3):
        curX = x1.copy()
        curX[d] += delta
        err = lsslam.linearize_pose_landmark_constraint(curX, x2, z)[0]
        curX[d] -= 2*delta
        err -= lsslam.linearize_pose_landmark_constraint(curX, x2, z)[0]

        ANumeric[:, d] = scalar * err

    diff = ANumeric - A
    if (abs(diff)).max() > eps:
        print('Error in the Jacobian for x1')
        print('Your analytic Jacobian', A)
        print('Numerically computed Jacobian', ANumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x1 appears to be correct')

    # test for x2
    BNumeric = scipy.zeros((2,2))
    for d in range(2):
        curX = x2.copy()
        curX[d] += delta
        err = lsslam.linearize_pose_landmark_constraint(x1, curX, z)[0]
        curX[d] -= 2*delta
        err -= lsslam.linearize_pose_landmark_constraint(x1, curX, z)[0]

        BNumeric[:, d] = scalar * err
        
    diff = BNumeric - B
    if (abs(diff)).max() > eps:
        print('Error in the Jacobian for x2')
        print('Your analytic Jacobian', B)
        print('Numerically computed Jacobian', BNumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x2 appears to be correct')
