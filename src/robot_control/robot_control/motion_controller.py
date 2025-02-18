#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from lidar_msg.msg import LidarData, LidarPoint
from lidar_msg.srv import StopRobot

import numpy as np

class motion_controller(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.subscription = self.create_subscription(
            LidarData,
            'lidar_data/a3',
            self.lidar_data_callback,
            10
        )
        self.subscription

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.stop_robot_server = self.create_service(
            StopRobot, 
            'stop_robot', 
            self.stop_robot_callback
        )

        self.MAX_LINEAR_SPEED = 0.2     # max linear speed of turtlebot3 burger
        self.MAX_ANGULAR_SPEED = 2.8    # max angular speed of turtlebot3 burger

        self.MIN_LIDAR_DIATANCE = 0.20
        self.MAX_LIDAR_DISTANCE = 0.40

        self.is_stopped = False
        
        self.get_logger().info('Motion controller node is running')

    def stop_robot_callback(self, request, response):
        self.is_stopped = request.stop
        response.success = True
        return response
    
    def lidar_data_callback(self, msg):
        cmd_vel = Twist()

        distances = np.array([point.distance for point in msg.scan_points])
        angles = np.array([point.angle for point in msg.scan_points])
        
        if not self.is_stopped:
            min_distance = np.min(distances) if len(distances) > 0 else float('inf')
            min_angle = angles[np.argmin(distances)] if len(distances) > 0 else 0
            
            if self.MIN_LIDAR_DIATANCE <= min_distance <= self.MAX_LIDAR_DISTANCE:
                speed_factor = (0.4 - min_distance) / (0.4 - 0.2)

                cmd_vel.linear.x = speed_factor * np.cos(min_angle) * self.MAX_LINEAR_SPEED
                cmd_vel.angular.z = -1 * speed_factor * np.sin(min_angle) * self.MAX_ANGULAR_SPEED

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = motion_controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()