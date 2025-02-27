#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lidar_msg.msg import LidarData, LidarPoint
from lidar_msg.srv import StopRobot
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('motion_collision')

        self.zone_access = np.ones(8, dtype=bool)
        self.prev_zone_access = np.ones(8, dtype=bool)

        self.OBSTACLE_DISTANCE = 0.20
        self.NUMBER_OF_NEAR_ZONES = 0

        

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
        self.stop_robot_client
        
        while not self.stop_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')
        
        self.get_logger().info('Obstacle Detector node has been started')

    def get_zones(self, angles):
        angles = np.mod(angles, np.pi*2)
        angles = (angles // (np.pi*45/180)).astype(int)
        return np.unique(angles)

    def get_adjacent_zones(self, zones):
        zones = np.array(zones)

        prev_zones = [(zones - i) % 8 for i in range(1, self.NUMBER_OF_NEAR_ZONES + 1)]
        next_zones = [(zones + i) % 8 for i in range(1, self.NUMBER_OF_NEAR_ZONES + 1)]
        
        all_zones_arrays = prev_zones + [zones] + next_zones
        all_zones = np.unique(np.concatenate(all_zones_arrays))
        return all_zones


    def lidar_data_callback(self, msg):
        distances = np.array([point.distance for point in msg.scan_points])
        angles = np.array([point.angle for point in msg.scan_points])
        
        obstacle_mask = distances < self.OBSTACLE_DISTANCE

        if np.any(obstacle_mask):

            obstacle_angles = angles[obstacle_mask]
            obstacle_zones = self.get_zones(obstacle_angles)
            
            zones_to_block = self.get_adjacent_zones(obstacle_zones)

            self.zone_access[zones_to_block] = False
            if not np.array_equal(self.zone_access, self.prev_zone_access):
                self.robot_command()
                self.prev_zone_access = self.zone_access.copy()

        else:
            self.zone_access = np.ones(8, dtype=bool)
            if not np.array_equal(self.zone_access, self.prev_zone_access):
                self.robot_command()
                self.prev_zone_access = self.zone_access.copy()

    def robot_command(self):
        request = StopRobot.Request()
        request.zone_access = self.zone_access.tolist()
        
        self.get_logger().info(f'Sending command: zone_access={request.zone_access}')
        
        future = self.stop_robot_client.call_async(request)
        future.add_done_callback(self.robot_command_callback)

    def robot_command_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f'Service call succeeded: {response.success}')
            else:
                self.get_logger().error('Service call returned None')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    obstacle = ObstacleDetector()
    rclpy.spin(obstacle)
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()