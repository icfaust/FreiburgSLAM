import scipy
import matplotlib.pyplot as plt
import gridmap

################################
#       Plotting Scripts       #
################################


def plot_map(mapout, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t):

    #figure(1, "visible", "off");
    plt.axis(mapBox);
    mapout = mapout.T
    
    plt.imshow(scipy.ones(mapout.shape) - log_odds_to_prob(mapout))
    s = mapout.shape[1] 
    #set(plt.gcf(), "position", [50 50 s*5]) 
    plt.subplots_adjust(.05, .05, .9, .9) 
    traj = scipy.vstack([poses[0:t, 0].T,poses[0:t, 1].T])
    traj = gridmap.world_to_map_coordinates(traj, gridSize, offset)
    plt.plot(traj[0],traj[1],'g')
    plt.plot(robPoseMapFrame[0], robPoseMapFrame[1], 'bo', markersize=5., linewidth=4., fillstyle='none')
    plt.plot(laserEndPntsMapFrame[0],laserEndPntsMapFrame[1],'ro',markersize=2., fillstyle='none')
    #filename = 'gridmap_%03d.png'.format(t)
    #plt.savefig(filename)

def plot_state(particles, timestep):
    """ Visualizes the state of the particles"""

    plt.grid("on")
    
    # Plot the particles
    ppos = scipy.array([p['pose'] for p in particles])
    plt.plot(ppos[:,0], ppos[:,1], 'g.', markersize=10., linewidth=3.5);

    plt.title('t= '+str(timestep))
    plt.xlim([-2, 12])
    plt.ylim([-2, 12])
    
    #dump to a file or show the window
    #window = False
    window = True
    if window:
        plt.pause(.5)
    else:
        plt.draw()
        filename = 'pf_%03d.png'.format(timestep)
        plt.savefig(filename)
