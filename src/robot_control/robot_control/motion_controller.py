#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from lidar_msg.msg import LidarData, LidarPoint
from lidar_msg.srv import StopRobot
import matplotlib.pyplot as plt
import numpy as np

class MotionControl(Node):
    def __init__(self):
        super().__init__('motion_controller')

        self.MAX_LINEAR_SPEED = 0.2     # max linear speed of turtlebot3 burger
        self.MAX_ANGULAR_SPEED = 1    # max angular speed of turtlebot3 burger

        self.MIN_LIDAR_DISTANCE = 0.20
        self.MAX_LIDAR_DISTANCE = 0.40

        self.zone_access = np.ones(8, dtype=bool)

        self.lidar_subscription = self.create_subscription(
            LidarData,
            'lidar_data/a3',
            self.lidar_data_callback,
            10
        )
        self.lidar_subscription

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
       
        # ตั้งค่า matplotlib แบบ non-blocking
        plt.ion()  # เปิดโหมด interactive
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        plt.tight_layout()
        
        # สร้าง timer สำหรับการอัปเดตการแสดงผล
        self.timer = self.create_timer(0.01, self.update_visualization)

        self.get_logger().info('Motion controller node has been started')

    def stop_robot_callback(self, request, response):
        self.zone_access = np.array(request.zone_access)
        response.success = True
        return response
    
    def get_zones(self, angles):
        angles = np.mod(angles, np.pi*2)
        return (angles // (np.pi*45/180)).astype(int)

    def check_angle_access(self, angles):
        zone = self.get_zones(angles)
        return  self.zone_access[zone]
    
    def lidar_data_callback(self, msg):
        cmd_vel = Twist()

        distances = np.array([point.distance for point in msg.scan_points])
        angles = np.array([point.angle for point in msg.scan_points])
        min_distance = np.min(distances) if len(distances) > 0 else float('inf')
        min_angle = angles[np.argmin(distances)] if len(distances) > 0 else 0

        if self.check_angle_access(min_angle):

            if self.MIN_LIDAR_DISTANCE <= min_distance <= self.MAX_LIDAR_DISTANCE:
                speed_factor = (self.MAX_LIDAR_DISTANCE - min_distance) / (self.MAX_LIDAR_DISTANCE - self.MIN_LIDAR_DISTANCE)

                cmd_vel.linear.x = 1 * speed_factor * np.cos(min_angle) * self.MAX_LINEAR_SPEED
                cmd_vel.angular.z = -1 * speed_factor * np.sin(min_angle) * self.MAX_ANGULAR_SPEED

        self.cmd_vel_publisher.publish(cmd_vel)

    def update_visualization(self):
        """อัปเดตการแสดงผลแบบไม่บล็อกการทำงาน"""
        # เคลียร์กราฟเก่า
        self.ax.clear()
        
        # จำนวนโซน
        n = len(self.zone_access)
        
        # ขนาดของแต่ละส่วนเท่ากันหมด
        sizes = [1] * n
        
        # กำหนดสี (เขียว = True, แดง = False)
        colors = ['green' if val else 'red' for val in self.zone_access]
        
        # สร้างแผนภูมิวงกลม
        self.ax.pie(sizes, colors=colors, wedgeprops=dict(width=1, edgecolor='black'), startangle=90)
        
        # วาดเส้นแบ่งส่วน
        radius = 1
        
        # วาดเส้นแนวนอนและแนวตั้งผ่านจุดศูนย์กลาง
        self.ax.axhline(y=0, color='black')
        self.ax.axvline(x=0, color='black')
        
        # วาดเส้นทแยงมุม
        self.ax.plot([-radius/np.sqrt(2), radius/np.sqrt(2)], [-radius/np.sqrt(2), radius/np.sqrt(2)], 'k-')
        self.ax.plot([-radius/np.sqrt(2), radius/np.sqrt(2)], [radius/np.sqrt(2), -radius/np.sqrt(2)], 'k-')
        
        # วาดเส้นขอบวงกลม
        circle = plt.Circle((0, 0), radius, fill=False, color='black')
        self.ax.add_patch(circle)
        
        zone_angles = range(0,45*8,45)  # องศาของแต่ละโซน
        
        for i in range(n):
            angle_rad = (112.5 + zone_angles[i]) * (np.pi / 180)
            x = 0.7 * np.cos(angle_rad)
            y = 0.7 * np.sin(angle_rad)
            self.ax.text(x, y, str(i), fontsize=12, ha='center', va='center')

        # ตั้งค่าอัตราส่วนให้เท่ากัน
        self.ax.set_aspect('equal')
        
        # ลบแกน
        self.ax.axis('off')
        
        # ตั้งชื่อ
        self.ax.set_title("Zone Access Status")
        
        # อัปเดตกราฟแบบไม่บล็อก
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    controller = MotionControl()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()