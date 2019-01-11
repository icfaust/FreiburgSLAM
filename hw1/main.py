import scipy

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
    output = {'id':data[0,:] - 1,
              'x':data[1,:],
              'y':data[2,:]}
    return output
    
################################
#      Old Data Objects        #
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

    
    
    
