# Extended Kalman Filter Project

This implements an Extended Kalman Filter in C++ and runs it agains a simulation of a radar and lidar measurements of a car as it travels around our frame of reference.  The image below shows the screen shot of the simulator.  The car is the car that is being tracked.  The red circles are radar measurements, the blue circles are lidar measurements, and the green triangles are the Extended Kalman Filter estimates.  

<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/advanced_lane_line_detection/blob/master/output_images/cc_initial_checkerboard_image.jpg" width="320px" /><br /><b>Test image taken with camera</b></p>
<br />

The simulator has 2 different simulations datasets it generates - Data Set 1 and Data Set 2 - each of these being a different path for the car.  The simulator also provides ground truth for calculating errors.  The program connects to the simualtor via raw sockets - port 4567. 

The Extended Kalman Filter generates estimates of the car's position and velocity which are better than the measurements provided by either the rador or lidar.  You can see this visually in the screen shot of the simulator above.   The green triangles representing the Extended Kalman Filter estimates are more of a smooth curve which is how we would expect a vehicle to traverse naturally, while the red and blue cirlces of the radar and lidar measurements appear more noisy. 




## Results

Since the simulator provides ground truth values, I was able to calculate errors for the estimates as mean squared error - MSE.
Running the code against Data Set 1, I recorded the following MSE's for x position (Px), y position (Py), velocity in the x diretion (Vx), and velocity in the y direction (Vy).

Variable | Measure
-----|-----
Px | 0.0964
Py | 0.0853
Vx | 0.5154
Vy | 0.4316

For Data Set 2, I recorded the following errors.

Variable | Measure
-----|-----
Px | 0.0727
Py | 0.0968
Vx | 0.4893
Vy | 0.5078

Going further, I recorded errors that considered lidar measurements only, ignoring the radar measurements.

Variable | Measure
-----|-----
Px | 0.1794
Py | 0.1533
Vx | 0.5942
Vy | 0.4724

Finally, I recorded errors for a run where the lidar was processed as normal and the radar measurements were converted to cartesian coordinates and processed ans lidar measurements.

Variable | Measure
-----|-----
Px | 0.1506
Py | 0.1660
Vx | 0.6012
Vy | 0.6089


