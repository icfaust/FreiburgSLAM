import scipy


def build_structure(g):#THIS FUNCTION NEEDS TO BE THOROUGHLY TESTED
    """ calculates the non-zero pattern of the Hessian matrix of a given graph"""
    idx = []

    # elements along the diagonal
    for value in g['idLookup']:# [value, key] = g.idLookup
        dim = value['dimension']
        offset = value['offset']
        r,c = scipy.mgrid[offset:offset+dim+1, offset:offset+dim+1]
        idx +=[r.flatten(), c.flatten()]


        # off-diagonal elements
        for eid in range(len(g['edges'])):# = 1:length(g.edges)
            edge = g['edges'][eid]
            if edge['type'] == 'P':
                r,c = scipy.mgrid[edge['fromIdx']:edge['fromIdx']+2, edge['toIdx']:edge['toIdx']+3]
                idx += [r.flatten(), c.flatten(), c.flatten(), r.flatten()]
            elif edge['type'] == 'L':
                r,c = scipy.mgrid[edge['fromIdx']:edge['fromIdx']+2, edge['toIdx']:edge['toIdx']+2]
                idx += [r.flatten(), c.flatten(), c.flatten(), r.flatten()]
                
    idx = scipy.concatenate(idx) #check this
    return idx

def get_block_for_id(g, idx):
    """returns the block of the state vector which corresponds to the given idx"""

    blockInfo = g['idLookup'][idx]
    block = g['x'][blockInfo['offset'] : blockInfo['offset'] + blockInfo['dimension'] + 1]

    return block


def get_poses_landmarks(g):
    """extract the offset of the poses and the landmarks"""

    poses = []
    landmarks = []
    
    for value in g['idLookup']:
        dim = value['dimension']
        offset = value['offset']
        if dim == 3:
            poses = scipy.array([poses, offset])
        elif dim == 2:
            landmarks = scipy.array([landmarks, offset])
  
    return poses, landmarks


def invt(m):
    """inverts a homogeneous transform"""
    A = scipy.eye((3,3))
    A[:2,:2]= m[:2, :2].T
    A[0:2, 2] = -1*scipy.dot(A[:2,:2], m[:2,2])
    return A

def nnz_of_graph(g):
    """calculates the number of non-zeros of a graph
    Actually, it is an upper bound, as duplicate edges
    might be counted several times"""

    nnz = 0

    # elements along the diagonal
    for value in g['idLookup']:
        nnz += pow(value['dimension'], 2)
    

    # off-diagonal elements
    for eid in range(len(g['edges'])):# 1:length(g.edges)
        edge = g['edges'][eid]
        if edge['type'] == 'P':
            nnz += 2 * 9
        elif edge['type'] == 'L':
            nnz += 2 * 6
    return nnz

        
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
