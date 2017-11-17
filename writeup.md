# Code Model

## main.cpp
This is where the program starts and communicates with the simulator. 
It makes use of the sensor_fusion object and trajectory object to help the car move.

## utilities.cpp
This file contains utility methods provided by Udacity.

## constants.h
This file contains project wide constants.
This was done to make tuning the project much easier.

## sensor_fusion.cpp
This class handles sensor fusion. 
It searches for obstacles in each lane and returns the best lane for the car to be in.
This is done using a cost function that penalizes speed on a lane basis.
Staying in the current lane is preferred over turning. 
Unless the car can go faster by changing lanes.
This class also makes sure that there is enough space for the car to turn.

##trajectory.cpp
This class handles trajectory generation.
It makes use of the spline library to generate a path.
This class does not make any decisions; it simply plans paths to a given lane.