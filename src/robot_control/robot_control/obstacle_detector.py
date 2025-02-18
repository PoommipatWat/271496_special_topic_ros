#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from lidar_msg.msg import LidarData, LidarPoint
from lidar_msg.srv import StopRobot

import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('motion_collision')

        self.lidar_subscription = self.create_subscription(
            LidarData,
            'lidar_data/turtlebot3',
            self.lidar_data_callback,
            10
        )
        self.lidar_subscription

        self.stop_robot_client = self.create_client(
            StopRobot,
            'stop_robot'
        )

        self.OBSTACLE_DISTANCE = 0.15

        self.obstacle_detected_status = False

        while not self.stop_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')

        self.get_logger().info('Obstable Detector node has been started')

    def lidar_data_callback(self, msg):
        distances = np.array([point.distance for point in msg.scan_points])

        if np.any(distances < self.OBSTACLE_DISTANCE) and self.obstacle_detected_status == False:
            self.obstacle_detected_status = True
            self.robot_command('stop')

        elif np.all(distances >= self.OBSTACLE_DISTANCE) and self.obstacle_detected_status:
            self.obstacle_detected_status = False
            self.robot_command('start')

        if self.obstacle_detected_status:
            angle = np.degrees(msg.scan_points[np.argmin(distances)].angle)
            self.get_logger().info(f'Obstacle detected : distance = {np.min(distances):.4f} m : angle = {angle:.4f} rad')

    def robot_command(self, command):
        request = StopRobot.Request()

        if command == 'start':
            request.stop = False
        elif command == 'stop':
            request.stop = True

        future = self.stop_robot_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Service call success')
        else:
            self.get_logger().error('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    obstacle = ObstacleDetector()
    rclpy.spin(obstacle)
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    


