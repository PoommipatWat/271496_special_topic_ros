# Final Project

This repository contains my final project for the 271496 Special Topics in Robotics Engineering 3 (ROS) course at Department of Robotics Engineering and AI, Chiang Mai University.

Student: Poommipat Wattanaprasit (640610682)

## Project Assignment

The task is to develop a ROS-based control system for a Turtlebot3 robot with the following requirements:

1. Create a node to control robot movement using RPLidar:
   - The robot should respond to a 50mm wide object placed at specified distances
   - Robot speed should increase as the object gets closer to the RPLidar
   - No speed limit is set, but movement must be safe for the robot

2. Implement obstacle detection:
   - The robot must detect surrounding obstacles
   - For obstacles within 150mm:
     - A service server should command the robot to stop
     - Warning messages should display obstacle angle and distance

## Development Requirements
- Must create custom:
  - Packages
  - Messages
  - Nodes (with meaningful names)
  - Launch files for running all nodes simultaneously