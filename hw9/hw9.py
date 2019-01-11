import scipy
import matplotlib.pyplot as plt
import calibrate

# load the odometry measurements
odom_motions = scipy.loadtxt('odom_motions.dat')

# the motions as they are estimated by scan-matching
scanmatched_motions = scipy.loadtxt('scanmatched_motions.dat')

# create our measurements vector z
z = scipy.hstack((scanmatched_motions, odom_motions))

# perform the calibration
X = calibrate.ls_calibrate_odometry(z)
print('calibration result'),
print(X)

# apply the estimated calibration parameters
calibrated_motions = calibrate.apply_odometry_correction(X, odom_motions)

# compute the current odometry trajectory, the scanmatch result, and the calibrated odom
odom_trajectory = calibrate.compute_trajectory(odom_motions)
scanmatch_trajectory = calibrate.compute_trajectory(scanmatched_motions)
calibrated_trajectory = calibrate.compute_trajectory(calibrated_motions)

# plot the trajectories
plt.plot(odom_trajectory[:,0], odom_trajectory[:,1],label="Uncalibrated Odometry")
plt.plot(scanmatch_trajectory[:,0], scanmatch_trajectory[:,1], label="Scan-Matching")
plt.plot(calibrated_trajectory[:,1], calibrated_trajectory(:,2), label="Calibrated Odometry")
plt.legend()
plt.show()
#plt.savefig('odometry-calibration.png')
