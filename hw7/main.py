import scipy
import scipy.linalg

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
    
    x = round(mycoords[:, 0])
    y = round(mycoords[:, 1])
    steep = (abs(y[1]-y[0]) > abs(x[1]-x[0]))

    if steep:
        x, y = _swap(x,y)
        
    if x[0] > x[1]: 
        [x[0], x[1]] = _swap(x[0], x[1])
        [y[0], y[1]] = _swap(y[0], y[1])

    delx = x[1] - x[0]
    dely = abs(y[1] - y[0])
    error = 0
    x_n = x[0]
    y_n = y[0]
    
    if y[0] < y[1]:
        ystep = 1
    else:
        ystep = -1 

    for n in xrange(delx):
        if steep:
            X[n] = x_n
            Y[n] = y_n
        else:
            X[n] = y_n
            Y[n] = x_n

    x_n += 1
    error += dely
    if bitshift(error, 1) >= delx: #same as -> if 2*error >= delx, 
        y_n += ystep
        error -= delx

    temp = X
    X = Y
    Y = temp

    return X,Y
    
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


def robotlaser_as_cartesian(rl, maxRange=15, subsample=False):

    numBeams = len(rl.ranges)
    maxRange=scipy.min(maxRange, rl.maximum_range);
    # apply the max range
    idx = scipy.logical_and(rl.ranges < maxRange, rl.ranges>0)
    
    if subsample:
        idx[1:2:] = 0.

    angles = scipy.linspace(rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution, numBeams)[idx]
    points = scipy.array([rl.ranges(idx)*scipy.cos(angles),
                          rl.ranges(idx)*scipy.sin(angles),
                          scipy.ones(1, len(angles))])
    transf = v2t(rl.laser_offset)

    # apply the laser offset
    points = transf * points

    return points

def read_robotlaser(filename_):
    """Reads the robot laser readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A LaserData class which contains the cartesian data and id
        
    Raises:
        NameError: incorrect filepath

    """
    return RobotLaser(filename_)

class RobotLaser(object):


    def __init__(self, name):
        
        tempdata = scipy.genfromtxt(name, dtype='object')
        idx = tempdata[:,0] == 'ROBOTLASER1'
        tempdata = tempdata[idx, 1:]
        
        self.start = tempdata[:, 1].astype(float)
        self.ang_res = tempdata[:, 4].astype(float)
        self.max_range = tempdata[:, 5].astype(float)

        #next two are accuracies
        self._num_readings = tempdata[:, 7].astype(int)

        #force object to int necessary for slicing
        self.scan = []
        self.pose = scipy.zeros((len(idx), 3))
        self.laser_offset = scipy.zeros((len(idx), 3))
        self.t = scipy.zeros((len(idx),))
        
        for i in range(len(tempdata)):
            idx1 = self._num_readings[i] + 8
            self.scan += [tempdata[i,8:idx1].astype(float)]

            offset = int(tempdata[i, idx1]) + 1
            idx1 += offset
            self.pose[i] = tempdata[i, idx1 + 3:idx1 + 6].astype(float)
            self.laser_offset[i] = t2v(scipy.dot(scipy.linalg.inv(v2t(self._pose[i])),
                                        v2t(tempdata[i, idx1:idx1 + 3].astype(float))))
            self.t[i] = tempdata[i, idx1 + 11]


#### PLOTTING FUNCTIONS ####

def plot_map(mapout, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t):

    #	figure 
    figure(1, "visible", "off");
    plt.axis(mapBox);
    hold on;
    mapout = mapout.T
    imshow(ones(size(map)) - log_odds_to_prob(map))
    s = size(map)(1:2); 
    set(plt.gcf(), "position", [50 50 s*5]) 
    set(plt.gca(), "position", [.05 .05 .9 .9]) 
    traj = [poses(1:t,1)';poses(1:t,2)'];
    traj = world_to_map_coordinates(traj, gridSize, offset);
    plt.plot(traj(1,:),traj(2,:),'g')
    plt.plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',4)
    plt.plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',2)
    #filename = sprintf('../plots/gridmap_%03d.png', t);
    #print(filename, '-dpng');
    #close all;
