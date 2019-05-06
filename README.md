# smart-2
Repository for SMART 2.0 robot

#Instructions to run the robot

# To run the robot hardware just launch the bring_up - For using multiple robots change the "smart2_** to another number" 	
roslaunch smart2_bring_up smart_2

*MATLAB demo code inside the smart2_malab_bridge

**Currently you also need to change conde inside the hardware interface

TODO
*Fix Hardware interface to make it compatible with multiple robots
*Fix the odometry (Z-Axis) to include IMU and not just wheel encoder
*Fix the smart2 description to load the actual SMART2 .dae file (apparently we need to resize it using blender)
*Change the serial port connections from */dev/ttyUSB* to /dev/serial/by-path for the serial connections to not start randomly 
