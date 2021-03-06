import scipy
import scipy.linalg
import matplotlib.pyplot as plt

def bresenham(mycoords):
    """ BRESENHAM: Generate a line profile of a 2d image 
    using Bresenham's algorithm
    [myline,mycoords] = bresenham(mymat,mycoords,dispFlag)

    - For a demo purpose, try >> bresenham();
    
    - mymat is an input image matrix.
    
    - mycoords is coordinate of the form: [x1, y1; x2, y2]
    which can be obtained from ginput function
    
    Author: N. Chattrapiban
    
    Ref: nprotech: Chackrit Sangkaew; Citec
    Ref: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
    
    See also: tut_line_algorithm"""
    
    x = scipy.around(mycoords[:, 0]).astype(int)
    y = scipy.around(mycoords[:, 1]).astype(int)
    steep = (abs(y[1]-y[0]) > abs(x[1]-x[0]))

    if steep:
        x, y = _swap(x,y)
        
    if x[0] > x[1]: 
        x[0], x[1] = _swap(x[0], x[1])
        y[0], y[1] = _swap(y[0], y[1])

    delx = x[1] - x[0]
    dely = abs(y[1] - y[0])
    X = scipy.zeros((int(delx),),dtype=int)
    Y = scipy.zeros((int(delx),),dtype=int)
    error = 0
    x_n = x[0]
    y_n = y[0]
    
    if y[0] < y[1]:
        ystep = 1
    else:
        ystep = -1 

    for n in range(delx):
        if steep:
            X[n] = x_n
            Y[n] = y_n
        else:
            X[n] = y_n
            Y[n] = x_n

        x_n += 1
        error += dely
        if (error << 1) >= delx: #same as -> if 2*error >= delx, 
            y_n += ystep
            error -= delx

    temp = X
    X = Y
    Y = temp

    return X, Y
    
def _swap(s,t):
    # function SWAP
    q = t
    r = s

    return q,r


def v2t(p):
    """ Transforms cartesian coordinates into homogeneous coordinates
   
    Args:
        p (3x1 numpy array): the robot pose in cartesian [x, y, theta]
             
    Returns:
        M (3x3 numpy array): the robot pose in homogeneous [[R, t],[0, 0, 1]]
    """
    output = scipy.array([[scipy.cos(p[2]), -scipy.sin(p[2]), p[0]],
                          [scipy.sin(p[2]), scipy.cos(p[2]), p[1]],
                          [0., 0., 1.]])
    
    return output
    

def t2v(R):
    
    """ Transforms cartesian coordinates into homogeneous coordinates
   
    Args:
        M (3x1 numpy array): the robot pose in homogeneous [[R, t],[0, 0, 1]]
             
    Returns:
        p (3x1 numpy array): the robot pose in cartesian [x, y, theta]
    """
    return scipy.array([R[0,2],R[1,2],scipy.arctan2(R[1,0],R[0,0])])


def normalize_angle(inp):
    """casts all angles into [-pi to pi]
    
    Args:
        inp (numpy array or float): numeric with elements which are angles
    
    Returns:
        inp (numpy array or float): array or value between -pi and pi

    """
    return (inp + scipy.pi) % (2*scipy.pi) - scipy.pi


def robotlaser_as_cartesian(rl, maxRange=15, subsample=False):

    numBeams = len(rl['scan'])
    maxRange = scipy.where(maxRange > rl['max_range'], rl['max_range'], maxRange) #a min call
    # apply the max range
    idx = scipy.logical_and(rl['scan'] < maxRange, rl['scan'] > 0)
    
    if subsample:
        idx[1:2:] = 0.

    angles = scipy.linspace(rl['start'], rl['start'] + numBeams*rl['ang_res'], numBeams)[idx]
    points = scipy.vstack([rl['scan'][idx]*scipy.cos(angles),
                          rl['scan'][idx]*scipy.sin(angles),
                          scipy.ones(angles.shape)])

    transf = v2t(rl['laser_offset'])

    # apply the laser offset
    points = scipy.dot(transf, points)

    return points


################################
#    Data Reading Scripts      #
################################

def read_robotlaser(filename_):
    """Reads the robot laser readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A LaserData class which contains the cartesian data and id
        
    Raises:
        NameError: incorrect filepath

    """
   
    tempdata = scipy.genfromtxt(filename_, dtype='object')
    idx = tempdata[:,0] == 'ROBOTLASER1'
    tempdata = tempdata[idx, 1:]
    num = tempdata[:, 7].astype(int) 
    laserdata = []
        
    for i in range(len(tempdata)):
        tempdict= {'start':float(tempdata[i,1]),
                   'ang_res':float(tempdata[i,3]),
                   'max_range':float(tempdata[i,4]),
                   'scan':[],
                   'pose':scipy.zeros((3,)),
                   'laser_offset':scipy.zeros((3,)),
                   't':[]}
        
        idx1 = num[i] + 8
        tempdict['scan'] = tempdata[i,8:idx1].astype(float)
        
        offset = int(tempdata[i, idx1]) + 1
        idx1 += offset
        
        tempdict['pose'] = tempdata[i, idx1 + 3:idx1 + 6].astype(float)
        tempdict['laser_offset'] = t2v(scipy.dot(scipy.linalg.inv(v2t(tempdict['pose'])),
                                                 v2t(tempdata[i, idx1:idx1 + 3].astype(float))))
        tempdict['t'] = float(tempdata[i, idx1 + 11])
        
        laserdata += [tempdict]
        
    return laserdata


def read_data(filename_, flag=True):
    """Reads the odometry and sensor readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A FburgData class which contains the odometry
        and/or sensor data
        
    Raises:
        NameError: incorrect filepath

    """
    output = {'sensor':[],'odometry':[]}
        
    data = scipy.genfromtxt(filename_, dtype='object')
    idx = scipy.squeeze(data[:,0] == 'ODOMETRY')
    for inp in data[idx,1:].astype(float):
        output['odometry'] += [{'r1':inp[0],
                                    't':inp[1],
                                    'r2':inp[2]}]

    idxarray = scipy.where(idx)
    idxarray = scipy.append(idxarray,[len(idx)])
    for i in range(len(idxarray) - 1):
        temp = []
        
        for j in scipy.arange(idxarray[i] + 1, idxarray[i + 1]):
            temp += [{'id':int(data[j,1]) - 1,
                      'range':float(data[j,2]),
                      'bearing':float(data[j,3])}]
                
        output['sensor'] += [temp]
    return output

