This homework runs completes the correction step of a fastSLAM algorithm, a full SLAM path is shown.

Files of use:
* hw8.py - code to be run to test homework output

* main.py - contains the other portions of the slam algorithm, including resampling, the measurement_model, and the prediction steps. It also includes the data reading functions.

* plot.py - plots the particles, with a trajectory following the highest weight particle per step, the landmarks and 1 sigma ellipses.

* fastslam.py - The correction function implements the various necessary EKF parts needed for the particle weight generation. 