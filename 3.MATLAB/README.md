# Trajectory Recovery and Data Analyser
Data analyser written in matlab.

### Feature
- Trajectory recovery from the 3-axes accelerometer and 3-axes gyroscope data using double integration.
- Automatic plotting of the displacement, velocity and acceleration in both the body and the earth inertial frames.
- Automatic detection of the interesting section of the data while plotting.
- Full 3D graph of the total trajectory.
- Animation with 1/20 real-time speed of rocket orientation is evaluated.
- Automatic data save.
- Mode bits to turn on/off the features above for convenience.
- We have seen that the maximum deviation between the retrieved IMU data and the real-time measurement data of on-board barometer is less than 20 meters out of 860 meters in our last ten flights.

### Usage
Put the decoded file from the phase 2 in the same workspace with this matlab file, then run the code.  
It's that simple.

### Credit
[Jaerin Lee](https://github.com/ironjr)  
Lead Electronics and Software Developer  
SNU Rocket Team Hanaro