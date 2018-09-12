import scipy

###################
#   Problem Two   #
###################

def motion_command(x, u):
    """Updates the robot pose according to the motion model
    - The resulting plot displays the following information:
        the landmarks in the map (black +'s)
    - current robot pose (red)
    - observations made at this time step (line between robot
         and landmark)
   
    Args:
        x (3x1 numpy array): the robot pose [x, y, theta]
        u (FburgData class): odometry reading {'r1', 't', 'r2'}.
             Use u['r1'], u['t'], and u['r2'] to access 
             the rotation and translation values
             
    Returns:
        x (3x1 numpy array): the new robot pose [x, y, theta]
     """

    #TODO: update x according to the motion represented by u
    x[0] += u[1]*scipy.cos(x[2] + u[0])
    x[1] += u[1]*scipy.sin(x[2] + u[0])
    #TODO: remember to normalize theta by calling normalize_angle for x[2]
    x[2] = (((x[2] + u[0] + u[2])/scipy.pi + 1.) % 2. - 1.)*scipy.pi
    
    return x

###################
#  Problem Three  #
###################

def v2t(p):
    """ Transforms cartesian coordinates into homogeneous coordinates
   
    Args:
        p (3x1 numpy array): the robot pose in cartesian [x, y, theta]
             
    Returns:
        M (3x3 numpy array): the robot pose in homogeneous [[R, t],[0, 0, 1]]
    """
    theta = p[2]
    
    if not theta: #divide by zero avoidance
        theta = 1.
            
    output = scipy.array([[scipy.cos(p[2]), -scipy.sin(p[2]), p[0]/theta],
                          [scipy.sin(p[2]), scipy.cos(p[2]), p[1]/theta],
                          [0., 0., 1.]])
    
    return output
    

def t2v(R):
    
    """ Transforms cartesian coordinates into homogeneous coordinates
   
    Args:
        M (3x1 numpy array): the robot pose in homogeneous [[R, t],[0, 0, 1]]
             
    Returns:
        p (3x1 numpy array): the robot pose in cartesian [x, y, theta]
    """
    return scipy.squeeze(scipy.arctan2(R[1,0],R[0,0])*R[:,2])

# Answer b) : the difference p2-p1 can be transformed into a homogeneous
# transform using delta x, delta y and delta theta

# Answer c): the answer to b can be recast to answer this problem. Relative
# pose means that R is essentially the identity matrix in the homogenous
# transform and z=t. This the homogeneous transform is fully specified.  Thus:
#
#  [[0,1,2/pi],     [[4/pi],    [[2/pi],                  [[1],
#   [1,0,2/pi],  x   [0],    =   [6/pi], == (in (x,y))==   [3]]
#   [0,0,1]]         [1]]        [1]]
