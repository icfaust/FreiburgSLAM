import scipy
import scipy.linalg

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
    mu += [z.range(), z.bearing()]
    
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
    return (inp + scipy.pi % 2*scipy.pi) - scipy.pi


def read_data(filename_):
    """Reads the odometry and sensor readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A FburgData class which contains the odometry
        and/or sensor data
        
    Raises:
        NameError: incorrect filepath

    """
    return FburgData(filename_)


def read_world(filename_):
    """Reads the world odometry and sensor readings from a file.
    
    Args:
        filename_: string containing file location
    
    Returns:
        output: A WorldData class which contains the cartesian data and id
        
    Raises:
        NameError: incorrect filepath

    """
    return WorldData(filename_)


################################
#        Data Objects          #
################################

class FburgData(object):
    """Class containing odometry and sensor data in the Freiburg format
    

    """ 
    
    def __init__(self, name):
        # data loader
        tempdata = scipy.genfromtxt(name, dtype='object')
        self._data = tempdata[:,1:].astype(float)
        
        idx = scipy.squeeze(tempdata[:,0] =='ODOMETRY')
        self._odometry = {'r1':self._data[idx,0],
                          't':self._data[idx,1],
                          'r2':self._data[idx,2]} #dicts work best in this case
        self._sensor = []
        self._idx = None
        idxarray = scipy.where(idx)
        idxarray = scipy.append(idxarray,[len(idx)])
        
        # this was done purely to match the MATLAB code (which is suboptimal)
        for i in xrange(len(idxarray)-1):
            temp = []
            
            for j in scipy.arange(idxarray[i]+1,idxarray[i+1]):
                temp += [{'id':self._data[j,0].astype(int) - 1,
                          'range':self._data[j,1],
                          'bearing':self._data[j,2]}]
                
            self._sensor += [temp]  
        
        #for i in scipy.unique(tempdata[idx,1].astype(int)):
            # check first index of sensors for unique inputs and iterate over
        #    self._sensor += [self._data[idx][self._data[idx,0] == i]]     
            # find sensor data which matches said unique index, and store as
            # a list
        
    def __len__(self):
        return len(self._odometry['r1'])
        
    def timestep(self, t):
        self._idx = t
        return self
        
    @property
    def odometry(self):
        if self._idx is None:
            return self._odometry
        else:
            output = {'r1':self._odometry['r1'][self._idx],
                      't':self._odometry['t'][self._idx],
                      'r2':self._odometry['r2'][self._idx]}
            self._idx = None
            return output
        
    @property
    def sensor(self):
        if self._idx is None:
            return self._sensor
        else:
            output = self._sensor[self._idx]
            self._idx = None
            return output

        
class WorldData(object):
    """Class containing landmark data in the Freiburg SLAM course format
    

    """ 
    def __init__(self, name):
        # data loader
        self._data = scipy.genfromtxt(name, dtype=float).T
            
        self._id = self._data[0]
        self._x = self._data[1]
        self._y = self._data[2]
    
    def __call__(self, t):
        (t)
        return {'id':self._id[t],
                'x':self._x[t],
                'y':self._y[t]}
        
    def landmarks(self, t):
        return {'id':self._id[t],
                'x':self._x[t],
                'y':self._y[t]}
    
    @property
    def id(self):
        return self._id
    
    @property
    def x(self):
        return self._x
    
    @property
    def y(self):
        return self._y
    
