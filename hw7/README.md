This homework evaluates the entirety of an unscented kalman filter SLAM using the same datasets from hw1 and hw4.  The full non-linear nature are discretized and evaluated and then analyzed as a gaussian ensemble.


Files of use:
* hw7.py - code which tests the first section homework output (gridmap.py, csail.log)

* gridmap.py - contains the various logarithmic odds and probabilities framework that need to be completed for the first part of the hw7 homework.

* csail.log - datafile which contains a floor of the MIT CSAIL building.  

* hw72.py - code which tests the second section homework output (motion.py, odometry.dat)

* motion.py - used for hw72 to calculate random motion of particles

* odometry.dat - datafile which reads the odometry positions using main.read_data. There is no sensor data in this set.

* main.py - contains necessary operational functions for both of the homework sets (gridmap.py and motion.py), including the bresenham algorithm, v2t, t2v, angle normalization (like the others, named normalize_angle), robotlaser_as_cartesian for gridmap, read_robotlaser

* plot.py - plotting functions to display robot movement, covariances, landmarks and movement purposely modified to display the gridmap (using the plot_map function).