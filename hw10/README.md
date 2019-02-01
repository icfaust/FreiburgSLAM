This homework runs completes the a sparse online-like least squares SLAM algorithm

Files of use:
* hw10.py - code which must be imported containing two test functions and the function needed for the homework. The test functions hw10.test_jacobian_pose_pose and hw10.test_jacobian_pose_landmark test functions of lsslam.py, namely the linearize_pose_pose_constraint and linearize_pose_landmark_constraint respectively. The homework can be run by the function hw10.lsSLAM(loc), where loc is a string describing the pickle file containing the odometry and sensor information. The default is 'simulation-pose-pose.p', but can run 'intel.p', 'dlr.p', and 'simulation-pose-landmark.p'

* lsslam.py - contains all functions which must be completed for the homework. _lsSLAM is imported into hw10 as lsSLAM which can be run there to complete the homework. It contains other computational functions: compute_global_error, linearize_and_solve, linearize_pose_pose_constraint, and linearize_pose_landmark_constraint. Interestingly I would have programmed these functions differently, as the compute_global_error is slightly redundant when similar computations are completed in the constraint functions.

* main.py - contains important functions get_poses_landmarks, and homogeneous transform functions v2t and t2v. Other functions, vestiges of the octave code, are also included but are unnecessary.

* plot.py - plots the odometry and landmark positions with the function plot_graph.

* translator.py - This was the script which converted the original .mat files into the used pickle files.

* simulation-pose-pose.p - pickle file containing data for testing pose-pose least square SLAM optimization

* simulation-pose-landmark.p - pickle file containing data for testing pose-landmark least square SLAM optimization

* dlr.p - pickle file of the pose-pose and pose-landmark of ???

* intel.p - pickle file of the pose-pose path of an intel building floor