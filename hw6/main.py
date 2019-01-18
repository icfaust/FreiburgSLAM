import scipy
import scipy.linalg
import plot

def add_landmark_to_map(mu, sigma, z, mapout, Q, scale):
    """Add a landmark to the UKF.
       We have to compute the uncertainty of the landmark given the current state
       (and its uncertainty) of the newly observed landmark. To this end, we also
       employ the unscented transform to propagate Q (sensor noise) through the
       current state"""
    
    # For computing sigma
    # FIND OUT WHY THEY WERE USING A GLOBAL ---> global scale;

    #add landmark to the map
    mapout += [z.idx]
    # TODO: Initialize its pose according to the measurement and add it to mu
    
    # Append the measurement to the state vector
    mu += [z['range'], z['bearing']]
    
    # Initialize its uncertainty and add it to sigma
    sigma = scipy.linalg.block_diag(sigma, Q)
    
    # Transform from [range, bearing] to the x/y location of the landmark
    # This operation intializes the uncertainty in the position of the landmark
    # Sample sigma points
    sig_pnts_new = compute_sigma_points(mu, sigma, scale)
    # Normalize!
    sig_pnts_new[2,:] = normalize_angle(sig_pnts_new[2,:])
    # Compute the xy location of the new landmark according to each sigma point
    newX = sig_pnts_new[0,:] + sig_pnts_new[-2,:]*scipy.cos(sig_pnts_new[2,:] + sig_pnts_new[-1,:])
    newY = sig_pnts_new[1,:] + sig_pnts_new[-2,:]*scipy.sin(sig_pnts_new[2,:] + sig_pnts_new[-1,:])
    # The last 2 components of the sigma points can now be replaced by the xy pose of the landmark
    sig_pnts_new[-2,:] = newX
    sig_pnts_new[-1,:] = newY
    
    # Recover mu and sigma
    #n = len(mu)
    #lam = scale - n;
    w0 = 1 - len(mu)/scale #lam/scale;
    wm = [w0, scipy.tile(1/(2*scale), (1, 2*n))]
    
    # Theta should be recovered by summing up the sines and cosines
    cosines = scipy.sum(scipy.cos(sig_pnts_new[2,:])*wm)
    sines = scipy.sum(scipy.sin(sig_pnts_new[2,:])*wm)
    
    # recompute the angle and normalize it
    mu_theta = scipy.arctan2(sines, cosines);
    mu = scipy.sum(sig_pnts_new*scipy.tile(wm, (sig_pnts_new.shape[0], 1)), 1)
    mu[2] = mu_theta

    diff = sig_pnts_new - scipy.tile(mu, (1, sig_pnts_new.shape[1]))

    # Normalize!
    diff[2,:] = normalize_angle(diff[2,:])
    sigma = scipy.dot(scipy.tile(wm, (diff.shape[0], 1))*diff, diff.T)
    
  return mu, sigma, mapout


def compute_sigma_points(mu, sigma, scale):
    """Computes the 2n+1 sigma points according to the unscented transform,
       where n is the dimensionality of the mean vector mu.
       The sigma points should form the columns of sigma_points,
       i.e. sigma_points is an nx2n+1 matrix."""

   #global scale;
   
   # Compute sigma points
   sigmasqr = scipy.linalg.sqrtm(sigma)
   sigmasqr = scipy.sqrt(scale) * sigmasqr
   
   mureplicated = scipy.tile(mu, (1, len(mu)))
   sigma_points = scipy.concatenate([mu, mureplicated + sigmasqr, mureplicated - sigmasqr])
   
   return sigma_points


def normalize_angle(inp):
    """casts all angles into [-pi to pi]
    
    Args:
        inp (numpy array or float): numeric with elements which are angles
    
    Returns:
        inp (numpy array or float): array or value between -pi and pi

    """
    return (inp + scipy.pi % 2*scipy.pi) - scipy.pi


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
    for i in xrange(len(idxarray) - 1):
        temp = []
        
        for j in scipy.arange(idxarray[i] + 1, idxarray[i + 1]):
            temp += [{'id':int(data[j,1]) - 1,
                      'range':float(data[j,2]),
                      'bearing':float(data[j,3])}]
                
        output['sensor'] += [temp]
    return output

def read_world(filename_):
    """Reads the world definitionodometry and sensor readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A WorldData class which contains the cartesian data and id
        
    Raises:
        NameError: incorrect filepath

    """
    #instead of trying to match the matlab object, return a dict
    data = scipy.genfromtxt(filename_, dtype=float).T
    output = {'id':data[0,:] - 1,
              'x':data[1,:],
              'y':data[2,:]}
    return output


################################
#      plotting routines       #
################################

    
def plot_state(mu, sigma, landmarks, timestep, mapout, z, window)
    """ Visualizes the state of the UKF SLAM algorithm.
    
     The resulting plot displays the following information:
     - map ground truth (black +'s)
     - current robot pose estimate (red)
     - current landmark pose estimates (blue)
     - visualization of the observations made at this time step (line between robot and landmark)"""

    plt.clf()
    plt.grid("on")
    
    L = struct2cell(landmarks); 
    plot.draw_probe_ellipse(land(1:3), sigma[0:3,0:3], 0.95, 'r'); #I don't know how to fix this here, will have to come back an
    plt.plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'k+', markersize=10., linewidth=5.)

    for i in xrange(len(mapout)):
	plt.plot(mu[2*i+ 1],mu[2*i+ 2], 'bo', markersize=10., linewidth=5.)
	plot.draw_prob_ellipse(mu[2*i+ 1:2*i+ 3],
                               sigma[2*i+ 1:2*i+ 2,2*i+ 1:2*i+ 2],
                               0.95,
                               'b');

    for in xrange(len(z)):
	loc = scipy.where(mapout == z['id'][i])
	mX = mu[2*loc];
	mY = mu[2*loc + 1]
    	plt.plot([mu[0], mX],[mu[1], mY], color='k', linewidth=1.)

    plot.drawrobot(mu[0:2], 'r', 3, 0.3, 0.3)
    plt.xlim([-2, 12])
    plt.ylim([-2, 12])

    #plot(sig_pnts(1:2,1),sig_pnts(1:2,2),'gx','linewidth',3);

    if window:
        plt.pause(0.1);
    else:
        plt.draw()
        filename = '../plots/ukf_%03d.png'.format(timestep)
        plt.savefig(filename)



    
