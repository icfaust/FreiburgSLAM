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

    
    #TODO: remember to normalize theta by calling normalize_angle for x[2]
    
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
    pass
    

def t2v(R):
    
    """ Transforms cartesian coordinates into homogeneous coordinates
   
    Args:
        M (3x1 numpy array): the robot pose in homogeneous [[R, t],[0, 0, 1]]
             
    Returns:
        p (3x1 numpy array): the robot pose in cartesian [x, y, theta]
    """
    pass

