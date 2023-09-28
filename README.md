# MAVS-MATLAB Interface
The MSU Autonomous Vehicle Simulator (MAVS) is [simulation library for autonomous ground vehicles](https://www.cavs.msstate.edu/capabilities/mavs.php) (AGV). MAVS is optimized for simulating off-road AGV. MAVS lets you create sensors like lidar and cameras, attach them to a vehicle, and simulate autonomous navigation.

This package interfaces to MATLAB and allows you to use the MATLAB built-in autonomy libraries with the MAVS simulators. 

MAVS-MATLAB is free. The MAVS-MATLAB package is a limited version of the full C++ library. The full version of MAVS is free and open source for **non-commercial** use only. 

For commercial licenses, see:
 https://www.cavs.msstate.edu/capabilities/mavs_request.php

Currently tested only for Windows 10 and MATLAB R2022b, but should work on 
Windows 11 and MATLAB R2019a and newer (anything with clib).

Installation instructions:

1. Clone the repo: 
```
$git clone https://github.com/CGoodin/mavs-matlab.git
```
2. From the mavs-matlab directory, run LaunchMavs.m

When running for the first time, you may be asked to specify a new data 
path folder. If so, select *mavs-matlab/mavs/data*.