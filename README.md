# Surgio_Ramis
7DOF Robotic manipulator with Remote Center of Motion enforcement.
1st: forward_kine.m
Calculates the forward kinematics using DH parameters.

2nd: spot_testing.m
Generates Random angles to test the arm later.

3rd: surgiov1.m
Model of the robot with RCM enforcement.

4th: surgio_manual_control_v1.m
Control the above model with an Xbox One controller.

5th: surgio_v1_arduino.m
Calculate and send the WPM signals needed to simulate the above models orbit in a real life robotic manipulator with servos.

To run this you need MATLAB with the following packages:
Robotics system toolbox, MATLAB Support Package for Arduino Hardware and Simulink 3D Animation.

First install the robotics toolbox from Peter Corke(https://petercorke.com/toolboxes/robotics-toolbox/)
and run the startup_rvc.m file. 
Then you can run the rest.
