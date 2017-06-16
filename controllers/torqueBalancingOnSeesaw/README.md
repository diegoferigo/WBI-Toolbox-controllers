# README #

Structure of the repository:

- matlab
- simulink
- imuCalib

## matlab ##
Contains all the `.m` files: utility functions (quaternion, skew-symmetric matrices, etc) and the `main.m` file needed to simulate the system using one of the ODE MATLAB integrator.

## simulink ##
Contains the simulink diagram and the initialization file.
The initialization file automatically adds the `matlab` folder to the path (because it uses some functions)

## imuCalib ##
A Simulink model for verifying seesaw IMU measurements

## How to test it ##
Open gazebo. Pause the simulation. Add the `iCub on seesaw` model. Play the controller
