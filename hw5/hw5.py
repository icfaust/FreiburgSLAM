import scipy


def compute_sigma_points(mu, sigma, lamb, alpha, beta):
    """This function samples 2n+1 sigma points from the distribution given by mu
       and sigma according to the unscented transform, where n is the 
       dimensionality of mu. Each column of sigma_points should represent one 
       sigma point (i.e. sigma_points has a dimensionality of nx2n+1). The
       corresponding weights w_m and w_c of the points are computed using lamb,
       alpha, and beta: w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n]
       (i.e. each of size 1x2n+1) They are later used to recover the mean and 
       covariance respectively."""

    n = length(mu)
    sigma_points = scipy.zeros((n, 2*n + 1))
    
    #TODO: compute all sigma points
    
    
    #TODO compute weight vectors w_m and w_c
    
    return sigma_points, w_m, w_c

def recover_gaussian(sigma_points, w_m, w_c):
    """This function computes the recovered Gaussian distribution (mu and sigma)
       given the sigma points (size: nx2n+1) and their weights w_m and w_c:
       w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
       The weight vectors are each 1x2n+1 in size,
       where n is the dimensionality of the distribution.
    
       Try to vectorize your operations as much as possible"""
    
    # TODO: compute mu

    
    # TODO: compute sigma

    return mu, sigma

def transform(points):
    """This function applies a transformation to a set of points.
       Each column in points is one point, 
       i.e. points = [[x1, y1], [x2, y2], ...]
       Select which function you want to use by uncommenting it
       (deleting the corresponding %{...%})
       while keeping all other functions commented."""

    #####
    # Function 1 (linear)
    # Applies a translation to [x; y]
    points[0, :] = points[0, :] + 1
    points[1, :] = points[1, :] + 2
    #####

    #####
    # Function 2 (nonlinear)
    # Computes the polar coordinates corresponding to [x; y]
    x = points[0, :]
    y = points[1, :]
    r = scipy.sqrt(scipy.sum([pow(x, 2), pow(y, 2)]))
    theta = scipy.atan2(y, x)
    points = scipy.array([r, theta])
    #####

    #####
    # Function 3 (nonlinear)
    points = points*scipy.cos(points)*scipy.sin(points)
    #####

    return points
