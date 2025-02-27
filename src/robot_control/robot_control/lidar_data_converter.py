#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSLivelinessPolicy, QoSReliabilityPolicy

from lidar_msg.msg import LidarData, LidarPoint

import numpy as np

class LidarDataConverter(Node):
    def __init__(self):
        super().__init__('lidar_data_converter')
        
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription_turtlebot = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_turtlebot_callback,
            qos
        )
        self.subscription_turtlebot
        
        self.publisher_turtlebot = self.create_publisher(
            LidarData,
            'lidar_data/turtlebot3',
            10
        )

        self.subscription_a3 = self.create_subscription(
            LaserScan,
            'scan/a3',
            self.scan_a3_callback,
            qos
        )
        self.subscription_a3
        
        self.publisher_a3 = self.create_publisher(
            LidarData,
            'lidar_data/a3',
            10
        )
        
        self.get_logger().info('LidarDataConverter node has been started')

    def scan_turtlebot_callback(self, msg):
        lidar_data = LidarData()
        
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        
        for i in range(len(msg.ranges)):
            point = LidarPoint()
            point.angle = float(angles[i])
            point.distance = float(msg.ranges[i])

            lidar_data.scan_points.append(point)
            
        self.publisher_turtlebot.publish(lidar_data)

    def scan_a3_callback(self, msg):
        lidar_data = LidarData()
        
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, msg.angle_increment)
        
        for i in range(len(msg.ranges)):
            point = LidarPoint()
            point.angle = float(angles[i])
            point.distance = float(msg.ranges[i])

            lidar_data.scan_points.append(point)
            
        self.publisher_a3.publish(lidar_data)

def main(args=None):
    rclpy.init(args=args)
    converter = LidarDataConverter()
    rclpy.spin(converter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()