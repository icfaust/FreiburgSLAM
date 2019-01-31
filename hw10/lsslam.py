import scipy
import scipy.sparse
import pickle
import main
import plot


def _lsSLAM(loc='simulation-pose-pose.p'):
    # load the graph into the variable g
    # only leave one line uncommented
    
    # simulation datasets
    #loc='simulation-pose-pose.p'
    
    #loc='simulation-pose-landmark.p'
    
    # real-world datasets
    #loc='intel.p'
    #loc='dlr.p'
    g = pickle.load(open(loc, 'rb'))
    
    # plot the initial state of the graph
    plot.plot_graph(g, 0)
    
    print('Initial error %f' % compute_global_error(g))

    # the number of iterations
    numIterations = 100

    # maximum allowed dx
    eps = 1e-4

    # Error
    err = 0.

    # carry out the iterations
    for i in range(numIterations):# = 1:numIterations
        print('Performing iteration {:3d}'.format(i))
    
        dx = linearize_and_solve(g)

        # TODO: apply the solution to the state vector g['x']
        g['x'] += dx
        
        # plot the current state of the graph
        plot.plot_graph(g, i)
        
        err = compute_global_error(g)
        
        # Print current error
        print('Current error %f' % err)
        
        # TODO: implement termination criterion as suggested on the sheet
        if err < eps:
            break #not a fan of this, I would have used a while loop

    print('Final error %f' % err)


def compute_global_error(g):
    """ Computes the total error of the graph"""
    Fx = 0
    # Loop over all edges
    for edge in g['edges']:

        # pose-pose constraint
        if edge['type'] == 'P':
            x1 = main.v2t(g['x'][edge['fromIdx']:edge['fromIdx']+3])  # the first robot pose
            x2 = main.v2t(g['x'][edge['toIdx']:edge['toIdx']+3])      # the second robot pose
            #TODO compute the error of the constraint and add it to Fx.
            # Use edge['measurement'] and edge['information'] to access the
            # measurement and the information matrix respectively.
            Z = main.v2t(edge['measurement'])
            e = main.t2v(scipy.dot(scipy.linalg.inv(Z),
                                   scipy.dot(scipy.linalg.inv(x1),
                                             x2))))
            Fx += scipy.sum(pow(e,2))
            
        # pose-landmark constraint
        elif edge['type'] == 'L':
            x = g['x'][edge['fromIdx']:edge['fromIdx']+3]  # the robot pose
            l = g['x'][edge['toIdx']:edge['toIdx']+2]      # the landmark

            #TODO compute the error of the constraint and add it to Fx.
            # Use edge['measurement'] and edge['information'] to access the
            # measurement and the information matrix respectively.
            Z = main.v2t(edge['measurement'])
            

    return Fx


def linearize_and_solve(g):
    """ performs one iteration of the Gauss-Newton algorithm
    each constraint is linearized and added to the Hessian"""

    nnz = main.nnz_of_graph(g)
    dx = scipy.zeros(g['x'].shape)
    
    # allocate the sparse H and the vector b
    H = scipy.sparse.dok_matrix((len(g['x']), len(g['x']))) #u
    b = scipy.zeros((len(g['x']), 1))

    needToAddPrior = True

    # compute the addend term to H and b for each of our constraints
    print('linearize and build system')
    for edge in g['edges']:

        # pose-pose constraint
        if edge['type'] == 'P':
            # edge['fromIdx'] and edge['toIdx'] describe the location of
            # the first element of the pose in the state vector
            # You should use also this index when updating the elements
            # of the H matrix and the vector b.
            # edge['measurement'] is the measurement
            # edge['information'] is the information matrix
            x1 = g['x'][edge['fromIdx']:edge['fromIdx']+2]  # the first robot pose
            x2 = g['x'][edge['toIdx']:edge['toIdx']+2]      # the second robot pose
            
            # Computing the error and the Jacobians
            # e the error vector
            # A Jacobian wrt x1
            # B Jacobian wrt x2
            e, A, B = linearize_pose_pose_constraint(x1, x2, edge['measurement'])


            # TODO: compute and add the term to H and b


            if needToAddPrior:
                # TODO: add the prior for one pose of this edge
                # This fixes one node to remain at its current location
                H[0,0] += 1.
                
                needToAddPrior = False

        # pose-landmark constraint
        elif edge['type'] == 'L':
            # edge['fromIdx'] and edge['toIdx'] describe the location of
            # the first element of the pose and the landmark in the state vector
            # You should use also this index when updating the elements
            # of the H matrix and the vector b.
            # edge['measurement'] is the measurement
            # edge['information'] is the information matrix
            x1 = g['x'][edge['fromIdx']:edge['fromIdx']+2]  # the robot pose
            x2 = g['x'][edge['toIdx']:edge['toIdx']+1]      # the landmark
          
            # Computing the error and the Jacobians
            # e the error vector
            # A Jacobian wrt x1
            # B Jacobian wrt x2
            e, A, B = linearize_pose_landmark_constraint(x1, x2, edge['measurement'])


            # TODO: compute and add the term to H and b

    print('solving system')

    # TODO: solve the linear system, whereas the solution should be stored in dx
    # Remember to use the backslash operator instead of inverting H

    return dx



def linearize_pose_landmark_constraint(x, l, z):
    """Compute the error of a pose-landmark constraint
    x 3x1 vector (x,y,theta) of the robot pose
    l 2x1 vector (x,y) of the landmark
    z 2x1 vector (x,y) of the measurement, the position of the landmark in
    the coordinate frame of the robot given by the vector x
    
    Output
    e 2x1 error of the constraint
    A 2x3 Jacobian wrt x
    B 2x2 Jacobian wrt l"""
    e = scipy.zeros((2,1))
    A = scipy.zeros((2,3))
    B = scipy.zeros((2,2))
    # TODO compute the error and the Jacobians of the error
    X = main.v2t(x)
    R = X[:2,:2]
    e = scipy.dot(R, l - X[:2,2]) - z
    deriv = scipy.array([[0.,-1.],
                         [1.,0.]])
    dRdtheta = scipy.dot(deriv, R)
    A[:,:2] = -1*R.T
    A[:,2] = scipy.dot(dRdtheta.T, l - X[:2,2])

    B = R.T
    
    return e, A, B


def linearize_pose_pose_constraint(x1, x2, z):
    """Compute the error of a pose-pose constraint
    x1 3x1 vector (x,y,theta) of the first robot pose
    x2 3x1 vector (x,y,theta) of the second robot pose
    z 3x1 vector (x,y,theta) of the measurement
    
    You may use the functions v2t() and t2v() to compute
    a Homogeneous matrix out of a (x, y, theta) vector
    for computing the error.
    
    Output
    e 3x1 error of the constraint
    A 3x3 Jacobian wrt x1
    B 3x3 Jacobian wrt x2"""
    e = scipy.zeros((3,1))
    A = scipy.zeros((3,3))
    B = scipy.zeros((3,3))
    # TODO compute the error and the Jacobians of the error
    X1 = main.v2t(x1)  # the first robot pose
    X2 = main.v2t(x2)      # the second robot pose
    Z = main.v2t(z)

    R1 = X1[:2,:2]
    RZ = Z[:2,:2]
    deriv = scipy.array([[0.,-1.],
                         [1.,0.]]) #fun story, the derivative of the rotation
    # can be also valued from a proper matrix multiplication just as inv(R) = R.T
    # dR/dtheta = scipy.dot(deriv, R). Do the math on a piece of paper, it works
    dR1dtheta = scipy.dot(deriv, R1)
    
    e = main.t2v(scipy.dot(scipy.linalg.inv(Z),
                           scipy.dot(scipy.linalg.inv(X1),
                                     X2))))
    
    
    A[:2,:2] = -1*scipy.dot(RZ.T, R1.T)
    A[2,:2] = (scipy.dot(RZ.T, scipy.dot(dR1dtheta.T, X2[:2,2]-X1[:2,2]))).T #the last transpose matches shape
    A[2,2] = -1.

    B[:2,:2] = -1*A[:2,:2].copy() #just to be sure
    B[2,2] = 1.
    
    return e, A, B


