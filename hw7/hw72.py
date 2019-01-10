import scipy
import hw7
import main

# Load laser scans and robot poses.
#load("../data/laser")
laser = main.read_robotlaser('csail.log')

# Extract robot poses: Nx3 matrix where each row is in the form: [x y theta]
#poses = laser.pose;
#poses = reshape(poses,3,size(poses,2)/3)';

# Initial cell occupancy probability.
prior = 0.50
# Probabilities related to the laser range finder sensor model.
probOcc = 0.9
probFree = 0.35

# Map grid size in meters. Decrease for better resolution.
gridSize = 0.5

# Set up map boundaries and initialize map.
border = 30
robXMin = laser.pose[:,0].min()
robXMax = laser.pose[:,0].max()
robYMin = laser.pose[:,1].min()
robYMax = laser.pose[:,1].max()
mapBox = sciyp.array([robXMin - border,
                      robXMax + border,
                      robYMin - border,
                      robYMax + border])
offsetX = mapBox[0]
offsetY = mapBox[2]
mapSizeMeters = scipy.array([mapBox[1] - offsetX, mapBox[3] - offsetY])
mapSize = scipy.ceil(mapSizeMeters/gridSize)

# Used when updating the map. Assumes that prob_to_log_odds.m
# has been implemented correctly.
logOddsPrior = hw7.prob_to_log_odds(prior)

# The occupancy value of each cell in the map is initialized with the prior.
mapout = logOddsPrior*scipy.ones((mapSize, mapSize))
print('Map initialized. Map size:'),
print(mapout.shape)

# Map offset used when converting from world to map coordinates.
offset = scipy.array([offsetX, offsetY])

# Main loop for updating map cells.
# You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
for t in xrange(len(laser.pose)):
    # Robot pose at time t.
    robPose = scipy.array([laser.pose[t,0],
                           laser.pose[t,1],
                           laser.pose[t,2]])
	
    # Laser scan made at time t.
    sc = laser.scan[t]
    # Compute the mapUpdate, which contains the log odds values to add to the map.
    mapUpdate, robPoseMapFrame, laserEndPntsMapFrame = hw7.inv_sensor_model(mapout, sc, robPose, gridSize, offset, probOcc, probFree)
    
    mapUpdate -= logOddsPrior*scipy.ones(mapout.shape)
    # Update the occupancy values of the affected cells.
    mapout += mapUpdate
    
    # Plot current map and robot trajectory so far.
    hw7.plot_map(mapout, mapBox, robPoseMapFrame, laser.pose, laserEndPntsMapFrame, gridSize, offset, t)
    
