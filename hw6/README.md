This homework evaluates the entirety of an unscented kalman filter SLAM using the same datasets from hw1 and hw4.  The full non-linear nature are discretized and evaluated and then analyzed as a gaussian ensemble.


Files of use:
* hw6.py - code to be run to test homework output

* plot.py - plotting functions to display robot movement, covariances, landmarks and movement purposely modified to display the 

* ukf_slam.py - contains the prediction and correction functions for the unscented Kalman filter which must be filled in to complete the assigment

* main.py - contains necessary operational functions for the ukf slam, including angle normalization, sigma point generation and landmark addtion to the map