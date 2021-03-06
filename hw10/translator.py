#!/usr/bin/python
import scipy
import scipy.io
import pickle

def load(loc='simulation-pose-pose.dat'):
    """this code is used to translate the hw10 .mat files into usable python
    dictionaries and arrays"""
    temp = scipy.io.loadmat(loc)

    output = {'edges':[],'idLookup':{}}
    output['x'] = scipy.squeeze(temp['g'][0][0][0])
    
    edges = temp['g'][0][0][1]
    for p in edges:
        inp = p[0]
        output['edges'] += [{'type':str(inp[0][0]),
                             'from':int(inp[1][0][0]) - 1,
                             'to':int(inp[2][0][0]) - 1,
                             'measurement':scipy.squeeze(inp[3]),
                             'information':scipy.squeeze(inp[4]),
                             'fromIdx':int(inp[5][0][0]) - 1,
                             'toIdx':int(inp[6][0][0]) - 1}]
                            
    idlookup = temp['g'][0][0][2]
    idx = [int(x) for x in str(idlookup.dtype).split("'")[1::4]]
    idlookup = idlookup[0][0]
    for i in range(len(idx)):
        inp = idlookup[i][0][0]
        output['idLookup'][idx[i]] = {'offset':int(inp[0][0][0]),
                                      'dimension':int(inp[1][0][0])}
        
    return output

g = load()
pickle.dump(g, open("simulation-pose-pose.p", "wb"))

g = load("simulation-pose-landmark.dat")
pickle.dump(g, open("simulation-pose-landmark.p", "wb"))

g = load("dlr.dat")
pickle.dump(g, open("dlr.p", "wb"))

g = load("intel.dat")
pickle.dump(g, open("intel.p", "wb"))
