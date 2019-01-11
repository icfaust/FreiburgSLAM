import scipy
import matplotlib.pyplot as plt


def build_structure(g):#THIS FUNCTION NEEDS TO BE THOROUGHLY TESTED
    """ calculates the non-zero pattern of the Hessian matrix of a given graph"""
    idx = []

    # elements along the diagonal
    for value, key in g.idLookup:# [value, key] = g.idLookup
        dim = value.dimension
        offset = value.offset
        [r,c] = scipy.mgrid[offset:offset+dim+1, offset:offset+dim+1]
        idx +=[r.flatten(), c.flatten()]


        # off-diagonal elements
        for eid in xrange(len(g.edges)):# = 1:length(g.edges)
            edge = g.edges[eid]
            if edge.type == 'P':
                [r,c] = scipy.mgrid[edge.fromIdx:edge.fromIdx+2, edge.toIdx:edge.toIdx+3]
                idx += [r.flatten(), c.flatten(), c.flatten(), r.flatten()];
            elif edge.type == 'L':
                [r,c] = scipy.mgrid(edge.fromIdx:edge.fromIdx+2, edge.toIdx:edge.toIdx+2];
                idx += [r.flatten() c.flatten(), c.flatten(), r.flatten()]];
                
    idx = scipy.concatenate(idx)
    return idx

def get_block_for_id(g, idx):
    """returns the block of the state vector which corresponds to the given idx"""

    blockInfo = g.idLookup[idx]
    block = g.x[blockInfo.offset : blockInfo.offset + blockInfo.dimension + 1)

    return block


def get_poses_landmarks(g):
    """extract the offset of the poses and the landmarks"""

    poses = [];
    landmarks = [];
    
    for value, key in g.idLookup:
        dim = value.dimension;
        offset = value.offset;
        if dim == 3:
            poses = [poses; offset];
        elif dim == 2:
            landmarks = [landmarks; offset];
  
    return poses, landmarks


def invt(m):
    """inverts a homogeneous transform"""
    A = scipy.eye((3,3))
    A[:2,:2]= m[:2, :2].T
    A[0:2, 2] = -1*scipy.dot(A[:2,:2],m[:2,2])
    return A

def nnz_of_graph(g):
    """calculates the number of non-zeros of a graph
    Actually, it is an upper bound, as duplicate edges
    might be counted several times"""

    nnz = 0

    # elements along the diagonal
    for value, key in g.idLookup:
        nnz += value.dimension^2
    

    # off-diagonal elements
    for eid in xrange(len(g.edges)):# 1:length(g.edges)
        edge = g.edges(eid)
        if edge.type == 'P':
            nnz += 2 * 9
        elif edge.type == 'L':
            nnz += 2 * 6
    return nnz

def plot_graph(g, iteration = -1):
    """plot of a 2D SLAM graph"""
    plt.clf()

    p, l = get_poses_landmarks(g)

    if length(l) > 0:
        landmarkIdxX = l+1;
        landmarkIdxY = l+2;
        plt.plot(g.x[landmarkIdxX], g.x[landmarkIdxY], '.or', markersize=4.)

    if length(p) > 0:
        pIdxX = p+1;
        pIdxY = p+2;
        plt.plot(g.x[pIdxX], g.x[pIdxY], '.xb', markersize=4.)
        
        # draw line segments???
    if False:
        poseEdgesP1 = [];
        poseEdgesP2 = [];
        landmarkEdgesP1 = [];
        landmarkEdgesP2 = [];
        for eid in xrange(len(g.edges)):# = 1:length(g.edges)
            edge = g.edges(eid)
            if edge.type == 'P':
                poseEdgesP1 = [poseEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
                poseEdgesP2 = [poseEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
            elif edge.type == 'L':
                landmarkEdgesP1 = [landmarkEdgesP1, g.x(edge.fromIdx:edge.fromIdx+1)];
                landmarkEdgesP2 = [landmarkEdgesP2, g.x(edge.toIdx:edge.toIdx+1)];
      
        linespointx = [poseEdgesP1(1,:); poseEdgesP2(1,:)];
        linespointy = [poseEdgesP1(2,:); poseEdgesP2(2,:)];

        plot(linespointx, linespointy, "r");
            
    plt.draw()
    plt.pause(0.1)
    if (iteration >= 0):
        filename = '/lsslam_%03d.png'.format(iteration);
        #plt.savefig(filename)
    #plt.show()
        

def read_graph(filename):
""" read a g2o data file describing a 2D SLAM instance"""
fid = fopen(filename, 'r');

graph = struct (
  'x', [],
  'edges', [],
  'idLookup', struct
);

disp('Parsing File');
while true
  ln = fgetl(fid);
  if (ln == -1)
    break;
  end
  tokens = strsplit(ln, ' ', true);
  double_tokens = str2double(tokens);

  tk = 2;
  if (strcmp(tokens(1), 'VERTEX_SE2') != 0)
    id = int32(double_tokens(tk++));
    values = double_tokens(tk:tk+2)'; tk += 3;
    graph.idLookup = setfield(graph.idLookup, num2str(id), struct('offset', length(graph.x), 'dimension', length(values)));
    graph.x = [graph.x; values];
  elseif (strcmp(tokens(1), 'VERTEX_XY') != 0)
    id = int32(double_tokens(tk++));
    values = double_tokens(tk:tk+1)'; tk += 2;
    graph.idLookup = setfield(graph.idLookup, num2str(id), struct('offset', length(graph.x), 'dimension', length(values)));
    graph.x = [graph.x; values];
  elseif (strcmp(tokens(1), 'EDGE_SE2') != 0)
    fromId = int32(double_tokens(tk++));
    toId = int32(double_tokens(tk++));
    measurement = double_tokens(tk:tk+2)'; tk += 3;
    uppertri = double_tokens(tk:tk+5)'; tk += 6;
    information = [uppertri(1), uppertri(2), uppertri(3);
                   uppertri(2), uppertri(4), uppertri(5);
                   uppertri(3), uppertri(5), uppertri(6)];
    graph.edges = [graph.edges; struct(
      'type', 'P',
      'from', fromId,
      'to', toId,
      'measurement', measurement,
      'information', information)];
  elseif (strcmp(tokens(1), 'EDGE_SE2_XY') != 0)
    fromId = int32(double_tokens(tk++));
    toId = int32(double_tokens(tk++));
    measurement = double_tokens(tk:tk+1)'; tk += 2;
    uppertri = double_tokens(tk:tk+2)'; tk += 3;
    information = [uppertri(1), uppertri(2); uppertri(2), uppertri(3)];
    graph.edges = [graph.edges; struct(
      'type', 'L',
      'from', fromId,
      'to', toId,
      'measurement', measurement,
      'information', information)];
  end

end

% setup the index into the state vector
disp('Preparing helper structs');
for eid = 1:length(graph.edges)
  graph.edges(eid).fromIdx = getfield(graph.idLookup, num2str(graph.edges(eid).from)).offset + 1;
  graph.edges(eid).toIdx = getfield(graph.idLookup, num2str(graph.edges(eid).to)).offset + 1;
end

return graph


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
