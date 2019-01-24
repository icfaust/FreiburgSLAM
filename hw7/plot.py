import scipy
import matplotlib.pyplot as plt
import gridmap

################################
#       Plotting Scripts       #
################################


def plot_map(mapout, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t, window=True):

    plt.clf()
    #plt.axis(mapBox);
    mapout = mapout.T
    plt.imshow(scipy.ones(mapout.shape) - gridmap.log_odds_to_prob(mapout),vmin=0.,vmax=1.)
    s = mapout.shape[1] 
    #set(plt.gcf(), "position", [50 50 s*5]) 
    plt.subplots_adjust(.05, .05, .9, .9)
    traj = []
    for i in range(t+1):
        traj += [poses[i]['pose'][:2]]
    traj = scipy.array(traj).T
    traj = gridmap.world_to_map_coordinates(traj, gridSize, offset)
    plt.plot(traj[0], traj[1], 'g')
    plt.plot(robPoseMapFrame[0], robPoseMapFrame[1], 'bo', markersize=5., linewidth=4., fillstyle='none')
    plt.plot(laserEndPntsMapFrame[0],laserEndPntsMapFrame[1],'ro',markersize=2., fillstyle='none')
    if window:
        #plt.show()
        plt.pause(.1)
    else:
        filename = 'gridmap_%03d.png'.format(t)
        plt.savefig(filename)

def plot_state(particles, timestep, window=True):
    """ Visualizes the state of the particles"""

    plt.grid("on")
    
    # Plot the particles
    ppos = scipy.array([p['pose'] for p in particles])
    plt.plot(ppos[:,0], ppos[:,1], 'g.', markersize=10., linewidth=3.5);

    plt.title('t= '+str(timestep))
    plt.xlim([-2, 12])
    plt.ylim([-2, 12])
    
    #dump to a file or show the window
    if window:
        plt.pause(.1)
    else:
        plt.draw()
        filename = 'pf_%03d.png'.format(timestep)
        plt.savefig(filename)
