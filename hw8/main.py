import scipy
import scipy.stats

def measurement_model(particle, z):
    """ compute the expected measurement for a landmark
    and the Jacobian with respect to the landmark"""

    # extract the id of the landmark
    landmarkId = z['id'];
    # two 2D vector for the position (x,y) of the observed landmark
    landmarkPos = particle.landmarks(landmarkId).mu;
    
    # TODO: use the current state of the particle to predict the measurment
    landmarkX = landmarkPos[0]
    landmarkY = landmarkPos[1]
    
    expectedRange = scipy.sqrt(pow(landmarkX - particle.pose[0], 2) + pow(landmarkY - particle.pose[1], 2))
    expectedBearing = normalize_angle(scipy.arctan2(landmarkY-particle.pose[1], landmarkX-particle.pose[0]) - particle.pose[2])

    h = scipy.array([expectedRange, expectedBearing])
    
    # TODO: Compute the Jacobian H of the measurement function h wrt the landmark location
    H = scipy.zeros(2,2)

    H[0,0] = (landmarkX - particle.pose[0])/expectedRange
    H[0,1] = (landmarkY - particle.pose[1])/expectedRange
    H[1,0] = (particle.pose[1] - landmarkY)/pow(expectedRange, 2)
    H[1,1] = (landmarkX - particle.pose[0])/pow(expectedRange, 2)
    
    return h, H

def normalize_angle(inp):
    return (inp + scipy.pi % 2*scipy.pi) - scipy.pi

def prediction_step(particles, u, noise):
    """ Updates the particles by drawing from the motion model
      Use u['r1'], u['t'], and u['r2'] to access the rotation and translation values
     which have to be pertubated with Gaussian noise.
    The position of the i-th particle is given by the 3D vector
     particles['pose'][i] which represents [x, y, theta].
    
     noise parameters
     Assume Gaussian noise in each of the three parameters of the motion model.
     These three parameters may be used as standard deviations for sampling."""
    r1Noise = noise[0]
    transNoise = noise[1]
    r2Noise = noise[2]

    numParticles = len(particles)

    for i in xrange(numParticles):#1:numParticles

        # append the old position to the history of the particle
        particles['history'] += [particles['pose'][i]]
        
        # sample a new pose for the particle
        r1 = scipy.stats.norm.rvs(loc=u['r1'], scale=r1Noise)
        r2 = scipy.stats.norm.rvs(loc=u['r2'], scale=r2Noise)
        trans = scipy.stats.norm.rvs(loc=u['t'], scale=transNoise)
        particles['pose'][i,0] = particles['pose'][i,0] + trans*scipy.cos(particles['pose'][i,2] + r1)
        particles['pose'][i,1] = particles['pose'][i,1] + trans*scipy.sin(particles['pose'][i,2] + r1)
        particles['pose'][i,2] = normalize_angle(particles['pose'][i,2] + r1 + r2)
        
    return particles


################################
#    Data Reading Scripts      #
################################


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
    output = {'id':data[0,:],
              'x':data[1,:],
              'y':data[2,:]}
    return output
