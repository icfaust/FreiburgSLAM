This is my interpretation of the 2013/2014 Freiburg homework sheet 1.
This has a rather un-pythonic implementation of the sensor system to 
best match the Octave scripts.  Some of this may be modified for
use on future homework assigments.

Files of use:
hw1.py - code to be run to test homework output

main.py - loads the datasets in the octave style (with some pythonic things)
  this replaces the tools/ framework of the original octave implementation

motion_command.py - python file with function motion_command
  which is used in question 2
  
plot.py contains the plotting routines called in hw1 guided by objects
  generated in main.py - currently does not save pngs, but could be done
  at a later point (also to generate the video)
