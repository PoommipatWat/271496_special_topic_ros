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
        
        self.obstacle_detected_status = False
        self.prev_obstacle_detected_status = False

        self.OBSTACLE_DISTANCE = 0.15

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
        angles = np.mod(angles, 360)
        return (angles // 45).astype(int)

    def get_adjacent_zones(self, zones):
        zones = np.array(zones)
        prev_zones = (zones - 1) % 8
        next_zones = (zones + 1) % 8
        all_zones = np.unique(np.concatenate([prev_zones, zones, next_zones]))
        return all_zones

    def check_state_changed(self):
        zones_changed = not np.array_equal(self.zone_access, self.prev_zone_access)
        status_changed = self.obstacle_detected_status != self.prev_obstacle_detected_status
        return zones_changed or status_changed

    def update_previous_state(self):
        self.prev_zone_access = self.zone_access.copy()
        self.prev_obstacle_detected_status = self.obstacle_detected_status

    def lidar_data_callback(self, msg):
        distances = np.array([point.distance for point in msg.scan_points])
        angles = np.degrees([point.angle for point in msg.scan_points])
        
        obstacle_mask = distances < self.OBSTACLE_DISTANCE
        if np.any(obstacle_mask):
            obstacle_angles = angles[obstacle_mask]
            obstacle_zones = self.get_zones(obstacle_angles)
            zones_to_block = self.get_adjacent_zones(obstacle_zones)
            
            new_zone_access = np.ones(8, dtype=bool)
            new_zone_access[zones_to_block] = False
            
            self.zone_access = new_zone_access
            self.obstacle_detected_status = True
            
            if self.check_state_changed():
                self.robot_command('stop')
                angle = np.degrees(msg.scan_points[np.argmin(distances)].angle)
                self.get_logger().info(f'Obstacle detected : distance = {np.min(distances):.4f} m : angle = {angle:.4f} rad')
            
            self.update_previous_state()
        else:
            self.zone_access = np.ones(8, dtype=bool)
            self.obstacle_detected_status = False
            
            if self.check_state_changed():
                self.robot_command('start')
            
            self.update_previous_state()

    def robot_command(self, command):
        request = StopRobot.Request()
        request.stop = (command == 'stop')
        future = self.stop_robot_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = 'success' if future.result() is not None else 'failed'
        response

def main(args=None):
    rclpy.init(args=args)
    obstacle = ObstacleDetector()
    rclpy.spin(obstacle)
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()