#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class DataAcquisitionNode(Node):
    def __init__(self):
        super().__init__('data_acquisition_node')
        
        # 1. Subscribe to Odometry Data (Position)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # 2. Subscribe to LiDAR Data (Range Scan)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
            
        self.get_logger().info('Data Acquisition System Initialized. Awaiting Sensor Streams...')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.get_logger().info(f"[ODOM DATA] -> X: {pos.x:.2f}m, Y: {pos.y:.2f}m")

    def laser_callback(self, msg):
        if len(msg.ranges) > 0:
            mid_index = len(msg.ranges) // 2
            front_distance = msg.ranges[mid_index]
            self.get_logger().info(f"[LIDAR DATA] -> Forward Obstacle Distance: {front_distance:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = DataAcquisitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
