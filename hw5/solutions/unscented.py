import scipy
import scipy.linalg

def compute_sigma_points(mu, sigma, lamb, alpha, beta):
    """This function samples 2n+1 sigma points from the distribution given by mu
       and sigma according to the unscented transform, where n is the 
       dimensionality of mu. Each column of sigma_points should represent one 
       sigma point (i.e. sigma_points has a dimensionality of nx2n+1). The
       corresponding weights w_m and w_c of the points are computed using lamb,
       alpha, and beta: w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n]
       (i.e. each of size 1x2n+1) They are later used to recover the mean and 
       covariance respectively."""

    n = len(mu)
    
    #TODO: compute all sigma points
    sigma_points = scipy.tile(mu, (2*n+1, 1)).T
    sqrt = scipy.linalg.sqrtm((n + lamb)*sigma)
    sigma_points[:,1:n+1] += sqrt
    sigma_points[:,n+1:] -= sqrt
    
    #TODO compute weight vectors w_m and w_c
    w_m = scipy.ones((2*n+1,))/(2*(n + lamb))
    w_c = w_m.copy()
    w_m[0] *= 2*lamb
    w_c[0] = w_m[0] + (1 - pow(alpha,2) + beta)
    
    return sigma_points, w_m, w_c

def recover_gaussian(sigma_points, w_m, w_c):
    """This function computes the recovered Gaussian distribution (mu and sigma)
       given the sigma points (size: nx2n+1) and their weights w_m and w_c:
       w_m = [w_m_0, ..., w_m_2n], w_c = [w_c_0, ..., w_c_2n].
       The weight vectors are each 1x2n+1 in size,
       where n is the dimensionality of the distribution.
    
       Try to vectorize your operations as much as possible"""
    
    # TODO: compute mu
    mu = scipy.dot(sigma_points, w_m)
    
    # TODO: compute sigma
    temp = sigma_points - scipy.tile(mu, (len(w_m), 1)).T
    sigma = scipy.dot(scipy.tile(w_c, (2, 1))*temp, temp.T)
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
    #####"""

    """#####
    # Function 2 (nonlinear)
    # Computes the polar coordinates corresponding to [x; y]
    x = points[0, :]
    y = points[1, :]
    r = scipy.sqrt(pow(x, 2) + pow(y, 2))
    theta = scipy.arctan2(y, x)
    points = scipy.vstack([r, theta])
    #####"""

    """#####
    # Function 3 (nonlinear)
    points = points*scipy.cos(points)*scipy.sin(points)
    #####"""

    return points
