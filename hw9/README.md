This homework runs completes an odometry calibration using the least-squares methodology described in the coursework, which minimizes the typically large inversion process

Files of use:
* hw9.py - code to be run to test homework output

* calibrate.py - functions to be completed for the homework, this includes everything necessary for the calibration scheme and trajectory generation (apply_odometry_correction, compute_trajectory, ls_calibrate_odometry, _error_function, and _jacobian)

* main.py - contains homogeneous coordinate transformers (v2t and t2v)

* scanmatched_motions.dat - A csv file containing the scanmatched odometry motions

* odom_motions.dat - A csxv file containing the raw odometry motions to be calibrated