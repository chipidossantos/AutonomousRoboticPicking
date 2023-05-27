# Autonomous Robotic Picking


## Project Description
Research project were I developed an Autonomous Robotic Picking system. It consists of an ABB IRB120 industrial robotic arm, an Intel Realsense D415 depth camera, and a software stack that performs computer vision for object position detection, mathematical calculations for frame tranformation, motion planning, and communication and control of the robot controller.

The system is built on a [ROS](https://www.ros.org/) architecture in order to integrate all software and hardware components. It uses the [abb_experimental](https://github.com/ros-industrial/abb_experimental) package to set up a client-server communication pathway between the ROS network and the robot controller, allowing it to command robot positions and waypoints as well as collect joint data. The system uses the [realsense-ros](https://github.com/IntelRealSense/realsense-ros) package in order to integrate the RGB and depth data from the camera into the ROS framework. For motion planning and visualization I am levaring the capabilities of [MoveIt](https://moveit.ros.org/) and [RVIZ](http://wiki.ros.org/rviz). Finally, the code running the system is modular in order to allow for easy integration of upgraded algorithms and all different software components communicate via a series of ROS topics and messages. 

## Code
The main code that runs the system is found under the [src](src) directory, while the [launch](launch) directory contains the necessary files to start up all system components and set up communication with the robot controller and camera. This is a brief description of the function each file performs:
* [src/depth_listener.py](src/depth_listener.py): receive depth data incoming from the camera and run the object detection algorithm to determine the object coordinates in 3D space with respect to the frame of reference of the camera, publish this position. Mark the determined object position on a live color feed of the camera.
* [src/transformation_calc.py](src/transformation_calc.py): perform mathematical calculation for transforming the object position from the camera frame to the robot frame of reference. This is done by using a transformation matrix tha was determined via a calibration procedure. Ensure robot is safe to move and publish final object position.
* [src/move_robot.py](src/move_robot.py): receive final object position, perform motion planning by using the move_group python interface for MoveIt, command motion of the robot and ensure motion plan was followed and joint positions are as expected.

## Photo
<img src="https://github.com/chipidossantos/AutonomousRoboticPicking/assets/95715590/e98f9e3e-e2f3-41b2-9354-e8ca8026825f" width="640" height="360">
