import scipy

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
