import scipy
from scipy.stats import chi2
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from matplotlib.patches import Rectangle
from matplotlib.patches import Arrow

def plot_state(mu, landmarks, timestep, z):
    """Visualizes the robot in the map.
    - The resulting plot displays the following information:
        the landmarks in the map (black +'s)
    - current robot pose (red)
    - observations made at this time step (line between robot
         and landmark)
   
    Args:
        mu (3x1 array): robot position and direction
        landmarks (WorldData obj): contains landmark
            positions
        timestep (integer): timestep index as an integer
        z (list): sensor data at timestep    
    """

    plt.grid('on')
    #figure(1, "visible", "off");
    plt.plot(landmarks.x, landmarks.y, 'k+', ms=10, lw=5.);

    for i in xrange(len(z)):
        id = z[i]['id']
        mX = landmarks(id)['x']
        mY = landmarks(id)['y']
        plt.plot([mu[0], mX],[mu[1], mY], color='b', lw=1.)

    drawrobot(mu[0:3], 'r', 3, 0.3, 0.3);
    plt.xlim([-2., 12.])
    plt.ylim([-2., 12.])
    plt.show()
    
    
    #modify this if you want to make the pngs and generate the video
    
    #filename = sprintf('../plots/odom_%03d.png', timestep);
    #print(filename, '-dpng');
    #hold off
    
def drawrobot(xvec, color, type=2, W=.2, L=.6):
    """Draws a robot at a set pose using matplotlib in current plot
   
    Args:
        xvec (3x1 array): robot position and direction
        color (string or rbg or rgba array): color following matplotlib specs      positions
    
    Kwargs:
        type (int [0:5]): dictates robot to be drawn with follow selections:
            - 0 : draws only a cross with orientation theta
            - 1 : draws a differential drive robot without contour
            - 2 : draws a differential drive robot with round shape
            - 3 : draws a round shaped robot with a line at theta
            - 4 : draws a differential drive robot with rectangular shape
            - 5 : draws a rectangular shaped robot with a line at theta
        W (float): robot width [m]    
        L (float): robot length [m]
    
    Returns:
        h (list): matplotlib object list added to current axes
    """
    
    theta = xvec[2]
    t = scipy.array([xvec[0], xvec[1]])
    r = []
    h = []
    
    if type ==0:
        cs = .1
        h += [plt.plot([cs,-cs,None,0.,0.]+t[0],
                       [0.,0.,None,cs,-cs]+t[1],
                       color,
                       lw=2.)]
    elif type == 1:
        xy = W*scipy.array((scipy.cos(theta + scipy.pi/2),
                            scipy.sin(theta + scipy.pi/2)))
        
        temp = Rectangle(t + xy, .03, .02, color=color, angle=theta)
        h += [plt.gca().add_artist(temp)]
        temp = Rectangle(t - xy, .03, .02, color=color, angle=theta)
        h += [plt.gca().add_artist(temp)]
        rin = _rot(theta,scipy.array([0, W + .03]))
        
        temp = Arrow(xvec[0] - rin[0],
                     xvec[1] - rin[1],
                     rin[0],
                     rin[1],
                     color=color)
        h += [temp]
        plt.gca().add_artist(temp)
        
    elif type == 2:
        xy = W*scipy.array((scipy.cos(theta + scipy.pi/2),
                            scipy.sin(theta + scipy.pi/2)))
        
        temp = Rectangle(t + xy, .03, .02, color=color, angle=theta)
        plt.gca().add_artist(temp)
        temp = Rectangle(t - xy, .03, .02, color=color, angle=theta)
        plt.gca().add_artist(temp)
        
        #axis between wheels here (requires a rotation)
        
        # The lines from the matlab come with no explanation, but do matrix
        #math to yield a rotated arrow
        rin = _rot(theta,scipy.array([0,W + .015]))
        
        temp = Arrow(xvec[0] - rin[0],
                     xvec[1] - rin[1],
                     rin[0],
                     rin[1],
                     color=color)
        plt.gca().add_artist(temp)
        
    elif type == 3:
        temp = Ellipse(xvec[:2],
                       W + .015,
                       W + .015,
                       angle=theta,
                       color=color)
        plt.gca().add_artist(temp)
        
        rin = _rot(theta,scipy.array([W + .015,0]))
        plt.plot(xvec[0]+scipy.array([-rin[0],rin[0]]),
                 xvec[1]+scipy.array([-rin[1],rin[1]]),
                 color=color,
                 lw=2.)
        
    elif type == 4:
        xy = W*scipy.array((scipy.cos(theta + scipy.pi/2),
                            scipy.sin(theta + scipy.pi/2)))
        
        temp = Rectangle(t + xy, .03, .02, color=color, angle=theta)
        plt.gca().add_artist(temp)
        h += [temp]
        
        temp = Rectangle(t - xy, .03, .02, color=color, angle=theta)
        plt.gca().add_artist(temp)
        h +=[temp]
                
        rin = _rot(theta,scipy.array([W + .015,0]))
        h += [plt.plot(xvec[0]+scipy.array([-rin[0],rin[0]]),
                       xvec[1]+scipy.array([-rin[1],rin[1]]),
                       color=color,
                       lw=2.)] 
        
        temp = Arrow(xvec[0] - rin[0],
                     xvec[1] - rin[1],
                     rin[0],
                     rin[1],
                     color=color)
        h += [temp]
                       
        temp = Rectangle(t, L, W, color=color, angle=theta)
        plt.gca().add_artist(temp)
        h +=[temp] 
        
        
    elif type == 5:
        rin = _rot(theta,scipy.array([W + .015,0]))
        h += [plt.plot(xvec[0]+scipy.array([-rin[0],rin[0]]),
                       xvec[1]+scipy.array([-rin[1],rin[1]]),
                       color=color,
                       lw=2.)] 
        
        temp = Arrow(xvec[0] - rin[0],
                     xvec[1] - rin[1],
                     rin[0],
                     rin[1],
                     color=color)
        h += [temp]
                       
        temp = Rectangle(t, L, W, color=color, angle=theta)
        plt.gca().add_artist(temp)
        h +=[temp] 
        
    else:
        raise ValueError('type out of bounds')

    
def draw_probe_ellipse(xy, covar, alpha, color=None, **kwargs):
    """Generates an ellipse object based of a point and related
    covariance assuming 2 dimensions
   
    Args:
        xy (2x1 array): (x,y) of the ellipse position
        covar (2x2 array): covariance matrix of landmark point
        alpha (float):
   
    Kwargs:   
        color (string): matplotlib color convention

    Returns:
         (matplotlib Ellipse Object): Ellipse object for drawing
 
    """
    
    b24ac = scipy.sqrt(pow(covar[0,0] - covar[1,1],2) + 4*pow(covar[0,1],2))
    c2inv = chi2.ppf(alpha, 2.)
    
    a = scipy.real(scipy.sqrt(c2inv*.5*(covar[0,0] + covar[1,1] + b24ac)))
    b = scipy.real(scipy.sqrt(c2inv*.5*(covar[0,0] + covar[1,1] - b24ac)))

    theta = .5*scipy.arctan2(2*covar[0,1], covar[0,0] - covar[1,1])
    
    if covar[1,1] > covar[0,0]:
        swap = a
        a = b
        b = swap

    ellipse = Ellipse(xy, 2*a, 2*b, angle=theta, edgecolor=color, fill=False, **kwargs)
    plt.gca().add_patch(ellipse)
    return ellipse


def _rot(theta, vec):
    """ there are a number of vector rotations in draw robot that are 
    not necessary to individually program.
    """

    rmat = scipy.array([[scipy.cos(theta), -1*scipy.sin(theta)],
                        [scipy.sin(theta), scipy.cos(theta)]]) 
    return scipy.dot(rmat,vec)
    
    

