This homework runs the transformation calculations needed for an unscented kalman filter. It shows the nature of descrete points in calculating SLAM.


Files of use:
* hw5.py - code to be run to test homework output

* plot.py - utilizes the draw_probe_ellipse function to give the 1 sigma contour

* unscented.py - Must complete the prediction and correction steps as described
  in the file.  The sigma points must be generated in compute_sigma_points, and the new mean vector and covariance matrix from transformed points is generated in recover_gaussian. The transformation of choice is dictated by the transform function, but must be done by changing what is commented-out