# ROS-Package-to-command-a-quadcopter-drone
This code can be implemented on a real machine,as well serving simulation purposes

To run it on a real machine you need an onboard computer ( Raspberry Pi ) and an autopilot like PIXHAWK with PX4 software and MavLink communication Protocol


For simulation, You need to run the drone simulation from the official Firmware/px4 repo ( forked in my profile) and then connect this model to it using MAVROS


when simulating the autonomous mode ( follow option ), you should be running the AI detection model which is in my other repository, and use your personal PC camera to show it different directions of lines.
