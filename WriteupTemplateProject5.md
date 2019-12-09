# [Extended Kalman Filter Project Starter Code]
Self-Driving Car Engineer Nanodegree Program

This project can also be found https://github.com/chaliburton/Udacity-ND-Self-Driving-Car-Project-5-Extended-Kalman-Filters-Project-
In this project I was provided with initial code for reading in measurements from Lidar and Radar sensors.  The starter code precluded functionality of the following code files:
1. FusionEKF.cpp
2. kalman_filter.cpp
3. tools.cpp

[image1]: ./examples/Fusion-Data-Set-1.JPG "Fusion - Data Set 1 Results"
[image2]: ./examples/Fusion-Data-Set-2.JPG "Fusion - Data Set 2 Results"
[image3]: ./examples/Radar-only-Data-Set-1.JPG "Radar - Data Set 1 Results"
[image4]: ./examples/Lidar-only-Data-Set-1.JPG "Lidar - Data Set 1 Results"


[FusionEKF.cpp]
This code defines the initial elements for the FusionEKF class and defines an object "ekf_" which is an onject of the kalman filter class as called in FusionEKF.h.  Note that when the initial state of the prediction is initialized the vx and vy velocities are initialized to zero.  The range rate is not a strong indicator of the actual object velocity.

[kalman_filter.cpp]
This code defines the matrix manipulation for the Kalman Filter or Extended Kalman Filter required for sensor fusion of the Lidar or Radar measurements. 
This code has the function for Predict, which estimates the state of the object being tracked and also Update &UpdateEKF, which updates the object state based on the applicable sensor measurement.  It's important to note that the sensor measurement added to the transformation vector can result in an updated theta outside of -pi through +pi and needs to be corrected. 

[tools.cpp]
This file defines two functions which are called on by the program. 
[[CalculateRMSE]] calculates the root mean squared error of the prediction from the Sensor-Fused EKF and the ground truth actual position.  This defines how accurate our EKF is at tracking the bicycle.

[CalculateJacobian] calculates the Jacobian matrix for the radar sensor which provides input in polar coordinates including range position, theta from dead-reckoning and range rate.  


# [For the Project Rubric:]
The code compiles without error.
The RMSE calculated meets specifications in that it is below the [0.11, .011, .052, 0.52]. 
Run forwards the RMSE was: [0.973, 0.0855, 0.4513, 0.4399] and can be seen in ![alt text][image1]
Run backwards the RMSE was: [0.0726, 0.0967, 0.4579, 0.4966] and can be seen in ![alt text][image2]

[Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.]
The code implemented follows the code developed throughout the lectures and follows the same defined set of steps.

[Your Kalman Filter algorithm handles the first measurements appropriately.]
Yes, the initial state was initialized with appropriate values.  Note that when the initial state of the prediction is initialized the vx and vy velocities are initialized to zero.  The range rate is not a strong indicator of the actual object velocity.

[Your Kalman Filter algorithm first predicts then updates.]
Yes, this was implemented as desired.

[Your Kalman Filter can handle radar and lidar measurements.]
Yes, this was implemented as desired.

#Code Efficiency
[Your algorithm should avoid unnecessary calculations]
Attempts were made to avoid unncessary calculations as can be observed in the code.  Specifically in the Jacobian matrix this can be seen to avoid multiple calculations.
Control Loops were minimized where possible.
The data structures were not expanded upon beyond what was initially provided
Control flow checks were not abundant as control loops were minimized and exited appropriately.

# [Suggestions to Make Your Project Stand Out!]
To inviestage how the radar and lidar impacted the the state prediction each was setup to be turned off.

The RMSE results for the Radar alone were: Forwards: [0.2238, 0.3461, 0.5671, 0.7877] Shown in ![alt text][image3]

The RMSE results for the Lidar alone were: Forwards: [0.1473, 0.1153, 0.6383, 0.5346] Shown in ![alt text][image4]

Compared to the results of the radar, the Lidar is more accurate at predicting location.  The fusion of both sensors results in a much better position and velocity estimation.  Neither sensor on it's own meets any of the given acceptability criteria. 
