import scipy
from main import read_world
from main import read_data
from plot import plot_state

# This script runs the main loop and calls all the required
# functions in the correct order.
#
#You can disable the plotting or change the number of steps the filter
# runs for to ease the debugging. You should however not change the order
# or calls of any of the other lines, as it might break the framework.
#
# If you are unsure about the input and return values of functions you
# should read their documentation which tells you the expected dimensions.
# 

#location of data sources
world_loc = 'world.dat'
data_loc = 'sensor_data.dat'

# Read world data, i.e. landmarks.
landmarks = read_world(world_loc)
# Read sensor readings, i.e. odometry and range-bearing sensor
data = read_data(data_loc)

# Initialize belief
# x: 3x1 vector representing the robot pose [x; y; theta]
x = scipy.zeros((3,))


# Iterate over odometry commands and update the robot pose
# according to the motion model
for t = xrange(len(data)):
    #distinctly python2.7

    # Update the pose of the robot based on the motion model
    x = motion_command(x, data.timestep(t).odometry);

    #Generate visualization plots of the current state
    plot_state(x, landmarks, t, data.timestep(t).sensor);

    print("Current robot pose:")
    print("x = ",x)

# Display the final state estimate
print("Final robot pose:")
print("x = ",x)