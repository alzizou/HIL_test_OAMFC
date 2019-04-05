# HIL_test_OAMFC
This folder includes the HIL test platform prepared to evaluate the Adaptive Model-Free Control (AMFC) algorithm for autonomous mobile robots.
The AMFC algorithm can be found in the following links:
1. (For SISO dynamic systems) https://ieeexplore.ieee.org/abstract/document/8272013 
2. (For MIMO dynamic systems) https://onlinelibrary.wiley.com/doi/abs/10.1002/acs.2865 

In this project, the performance of recently proposed adaptive model-free control (AMFC) algorithm for autonomous mobile robots 
is evaluated in a hardware-in-the-loop (HIL) test platform.
The algorithm is a solution for tracking problem in completely unknown nonlinear dynamic systems operated under unknown external 
disturbances. Here, the AMFC algorithm alongside with a standard Kalman-filter for eliminating the measurement noise is embedded 
on an Arduino-based microcontroller and the dynamic model of the autonomous mobile robot is implemented on another Arduino-based 
microcontroller. The data between two microcontollers is transferred using an I2C communication protocol. 
In addition, the data is logged during the HIL test in MATLAB using serial communication between the microcontrollers and a laptop. 
Appropriate performance of the AMFC algorithm is observed based on the results for the emulated dynamic models of a quadrotor 
and a wheeled mobile robot (WMR).

Read the provided PDF file for full report and results.

Also, the AMFC algorithm is prepared in OOP style for practical implementations in Arduino as the flight controller of a quadrotor and a WMR. See inside the folders with "OOP" in the name.
