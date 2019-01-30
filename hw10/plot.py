import matplotlib.pyplot as plt 
import main

def plot_graph(g, iteration = -1):
    """plot of a 2D SLAM graph"""
    plt.clf()

    p, l = main.get_poses_landmarks(g)

    if len(l) > 0:
        landmarkIdxX = l
        landmarkIdxY = l + 1
        plt.plot(g['x'][landmarkIdxX],
                 g['x'][landmarkIdxY],
                 'or',
                 markersize=4.,
                 fillstyle='none')

    if len(p) > 0:
        pIdxX = p
        pIdxY = p + 1
        plt.plot(g['x'][pIdxX], g['x'][pIdxY], 'xb', markersize=4.)
        
        # draw line segments???
    if False:
        poseEdgesP1 = []
        poseEdgesP2 = []
        landmarkEdgesP1 = []
        landmarkEdgesP2 = []
        for eid in range(len(g['edges'])):# = 1:length(g.edges)
            edge = g['edges'][eid]
            if edge['type'] == 'P':
                poseEdgesP1 = [poseEdgesP1, g['x'][edge['fromIdx']:edge['fromIdx']+1]]
                poseEdgesP2 = [poseEdgesP2, g['x'][edge['toIdx']:edge['toIdx']+1]]
            elif edge['type'] == 'L':
                landmarkEdgesP1 = [landmarkEdgesP1, g['x'][edge['fromIdx']:edge['fromIdx']+1]]
                landmarkEdgesP2 = [landmarkEdgesP2, g['x'][edge['toIdx']:edge['toIdx']+1]]
      
        linespointx = [poseEdgesP1[0], poseEdgesP2[0]]
        linespointy = [poseEdgesP1[1], poseEdgesP2[1]]

        plt.plot(linespointx, linespointy, "r")
            
    plt.draw()
    plt.pause(0.1)
    if (iteration >= 0):
        filename = '/lsslam_%03d.png'.format(iteration)
        #plt.savefig(filename)
    #plt.show()
