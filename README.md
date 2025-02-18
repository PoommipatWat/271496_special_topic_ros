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

Must create custom:
- Packages
- Messages
- Nodes (with meaningful names)
- Launch files for running all nodes simultaneously

## Installation

1. Clone the repository:
```bash
git clone https://github.com/PoommipatWat/640610682_final.git
```

2. Navigate to the workspace directory:
```bash
cd 640610682_final
```

3. Build the project:
```bash
colcon build
```

## Usage

1. Source the setup file:
```bash
source install/setup.bash
```

2. Launch the control system:
```bash
ros2 launch robot_control robot_control.launch.py
```

## Run Turtlebot3

1. SSH into the Raspberry Pi:
```bash
ssh pi@raspberrypi.local
```

2. Set the Turtlebot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

3. Launch the robot:
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

## Project Structure

```
640610682_final/
├── src/
│   ├── lidar_msg/             # Custom messages package
│   │   ├── msg/
│   │   │   ├── LidarData.msg
│   │   │   └── LidarPoint.msg
│   │   └── srv/
│   │       └── StopRobot.srv
│   └── robot_control/         # Main control package
│       ├── robot_control/
│       │   ├── lidar_data_converter.py 
│       │   ├── motion_collision.py
│       │   └── obstacle_detector.py
│       └── launch/
│           └── robot_control.launch.py
├── build/
├── install/
├── log/
└── README.md
```

## Nodes Description
1. `lidar_data_converter` node:
   - Converts raw LaserScan messages to custom LidarData format

2. `motion_collision` node:
   - Subscribes to LiDAR data
   - Detects obstacles within 150mm
   - Provides stop service
   - Handles zone-based access control

3. `motion_controller` node:
   - Controls robot movement
   - Responds to stop service
   - Adjusts speed based on object distance
   - Implements zone-based movement restrictions

## Custom Topics and Services

### Custom msg:
1. LidarData message:
   ```
   LidarPoint[] scan_points  # Array of LidarPoint messages
   ```

2. LidarPoint message:
   ```
   float32 angle     # Angle of the scan point in radians
   float32 distance  # Distance to obstacle in meters
   ```

### Custom srv:
- `/stop_robot`: Control robot movement and zone access
  ```
  # Request
  bool stop             # true = stop,    false = resume
  bool[8] zone_access   # true = access,  false = no access

  # Zone definitions:
  # zone 0 = [0,45] degree     # zone 4 = [180,225] degree
  # zone 1 = [45,90] degree    # zone 5 = [225,270] degree
  # zone 2 = [90,135] degree   # zone 6 = [270,315] degree
  # zone 3 = [135,180] degree  # zone 7 = [315,360] degree

  ---
  # Response
  bool success  # true if command was successful
  ```

## Author

Poommipat Wattanaprasit (640610682)
Department of Robotics Engineering and AI
Chiang Mai University