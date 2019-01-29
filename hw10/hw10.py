#!/usr/bin/python
import scipy
import pickle
import main


def lsSLAM():
    # load the graph into the variable g
    # only leave one line uncommented
    
    # simulation datasets
    loc='simulation-pose-pose.p'
    
    #loc='simulation-pose-landmark.p'
    
    # real-world datasets
    #loc='intel.p'
    #loc='dlr.p'
    g = pickle.load(open(loc, 'rb'))
    
    # plot the initial state of the graph
    main.plot_graph(g, 0)
    
    print('Initial error %f\n' % main.compute_global_error(g))

    # the number of iterations
    numIterations = 100

    # maximum allowed dx
    eps = 10^-4

    # Error
    err = 0.

    # carry out the iterations
    for i in range(numIterations):# = 1:numIterations
        print('Performing iteration 03%d\n'.format(i))
    
        dx = main.linearize_and_solve(g)

        # TODO: apply the solution to the state vector g.x
        
        # plot the current state of the graph
        main.plot_graph(g, i)
        
        err = main.compute_global_error(g)
        
        # Print current error
        print('Current error %f\n' % err )
        
        # TODO: implement termination criterion as suggested on the sheet


    print('Final error %f\n' % err)

    
def test_jacobian_pose_pose():

    eps = 1e-5;

    x1 = scipy.array([1.1, 0.9, 1])
    x2 = scipy.array([2.2, 1.85, 1.2])
    z  = scipy.array([0.9, 1.1, 1.05])

    # get the analytic Jacobian
    e, A, B = main.linearize_pose_pose_constraint(x1, x2, z)

    # check the error vector
    e_true = scipy.array([-1.06617,  -1.18076,  -0.85000])
    if norm(e - e_true) > eps:
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
    for d in range(3):# = 1:3
        curX = x1
        curX[d] += delta
        err = main.linearize_pose_pose_constraint(curX, x2, z)
        curX = x1
        curX[d] -= delta
        err -= main.linearize_pose_pose_constraint(curX, x2, z)
        
        ANumeric[:, d] = scalar * err


    diff = ANumeric - A
    if scipy.max(abs(diff))) > eps:
        print('Error in the Jacobian for x1')
        print('Your analytic Jacobian', A)
        print('Numerically computed Jacobian', ANumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x1 appears to be correct')

    #test for x2
    BNumeric = scipy.zeros((3,3))
    for d in range(3):# = 1:3
        curX = x2
        curX[d] += delta
        err = main.linearize_pose_pose_constraint(x1, curX, z)
        curX = x2
        curX[d] -= delta
        err -= main.linearize_pose_pose_constraint(x1, curX, z)

        BNumeric[:, d] = scalar * err


    diff = BNumeric - B;
    if scipy.max(abs(diff))) > eps:
        print('Error in the Jacobian for x2')
        print('Your analytic Jacobian', B)
        print('Numerically computed Jacobian', BNumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x2 appears to be correct')


def test_jacobian_pose_landmark():
    eps = 1e-5;

    x1 = scipy.array([1.1, 0.9, 1.])
    x2 = scipy.array([2.2, 1.9])
    z  = scipy.array([1.3, -0.4])

    # get the analytic Jacobian
    e, A, B = main.linearize_pose_landmark_constraint(x1, x2, z)

    # check the error vector
    e_true = scipy.array([0.135804, 0.014684])
    if norm(e - e_true) > eps:
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
    for d in range(3):#= 1:3
        curX = x1
        curX[d] += delta
        err = main.linearize_pose_landmark_constraint(curX, x2, z)
        curX = x1
        curX[d] -= delta
        err -= main.linearize_pose_landmark_constraint(curX, x2, z)

        ANumeric[:, d] = scalar * err

    diff = ANumeric - A
    if scipy.max(abs(diff)) > eps:
        print('Error in the Jacobian for x1')
        print('Your analytic Jacobian', A)
        print('Numerically computed Jacobian', ANumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x1 appears to be correct')

    # test for x2
    BNumeric = scipy.zeros((2,2))
    for d in range(2):# 1:2
        curX = x2
        curX[d] += delta
        err = main.linearize_pose_landmark_constraint(x1, curX, z)
        curX = x2
        curX[d] -= delta
        err -= main.linearize_pose_landmark_constraint(x1, curX, z)

        BNumeric[:, d] = scalar * err
        
    diff = BNumeric - B
    if scipy.max(abs(diff))) > eps:
        print('Error in the Jacobian for x2')
        print('Your analytic Jacobian', B)
        print('Numerically computed Jacobian', BNumeric)
        print('Difference', diff)
    else:
        print('Jacobian for x2 appears to be correct')
