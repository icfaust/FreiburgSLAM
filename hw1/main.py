import scipy
import scipy.io
from __builtin__ import int




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

class FburgData():
    """Class containing odometry and sensor data in the Freiburg format
    

    """ 
    
    def __init__(self, name):
        # data loader
        tempdata = scipy.readfromtxt(name, dtype='object')
        self._data = tempdata[:,1:].astype(float)
        
        idx = tempdata[:,0] == 'ODOMETRY'
        self._odometry = self._data[idx]
        self._sensor = []
        
        idx = tempdata[:,0] == 'SENSOR'
        for i in scipy.unique(tempdata[idx,1].astype(int)):
            # check first index of sensors for unique inputs and iterate over
            self._sensor += [self._data[idx][self._data[idx,0] == i]]     
            # find sensor data which matches said unique index, and store as
            # a list
        
    def timestep(self, t):
         