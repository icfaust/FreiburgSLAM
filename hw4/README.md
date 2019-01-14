This homework runs an extended Kalman fliter on the data similarly used in other
homeworks (such as hw1, with ../sensor_data.dat and ../world.dat). Different
from the assigment, all the work can be accomplished in this directory using
the modules included here.  The files are described below, which follow a
'pythonic' structure rather than the octave/MATLAB framework described in the
pdf.


Files of use:
* hw4.py - code to be run to test homework output

* main.py - loads the datasets in a pythonic way (loaded into dicts). This
  replaces the tools/ framework of the original octave implementation

* ekf_slam.py - Must complete the prediction and correction steps as described
  in the file
  
* plot.py contains the plotting routines called in hw4 guided by objects
  generated in main.py - currently does not save pngs, but could be done
  at a later point (also to generate the video)
