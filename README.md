# service-robot
Automation of a robot in an indoor environment to aid with chores

This code defines a ROS 2 Turtlebot, it controls the robot's movement using velocity commands while processing odometry and laser scan data. The node subscribes to the odometry to track the robot's position and orientation. The scan topic is used to analyze lidar laser sensor readings for obstacle detection. The robot moves through predefined stages using velocity commands, adjusting its movement based on detected landmarks and obstacles. The program also logs odometry, velocity, and laser scan data into readable CSV files for analysis. The EulerAngles and Quaternion structures help convert orientation data, and a transformMapPoint function calculates mapped coordinates from laser readings. The main function initializes the ROS node and starts the robot’s movement loop.

https://github.com/user-attachments/assets/bc364219-0c5a-47c6-8d39-c07b15dd5c41
