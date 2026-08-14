#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class InspectionManagerNode(Node):
    def __init__(self):
        super().__init__('inspection_manager_node')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/inspection/status', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.front_min_dist = 10.0
        
        # Inspection Route Waypoints (Industrial Floor Map Coordinates)
        self.waypoints = [
            {"name": "Inspection Checkpoint A (East Corridor)", "x": 1.5, "y": 0.0},
            {"name": "Inspection Checkpoint B (North Storage)", "x": 1.0, "y": 1.2},
            {"name": "Inspection Checkpoint C (West Chemical Area)", "x": -1.2, "y": 1.2},
            {"name": "Inspection Checkpoint D (South Equipment Bay)", "x": -1.2, "y": -1.2},
            {"name": "Inspection Checkpoint E (Central Hub - Return)", "x": 0.0, "y": 0.0}
        ]
        
        self.current_wp_idx = 0
        self.state = 'NAVIGATING'
        self.pause_start_time = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous Inspection Route Manager active with LiDAR Obstacle Avoidance...')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            # Check front 60-degree arc (-30 to +30 deg)
            total_samples = len(msg.ranges)
            center = total_samples // 2
            arc_samples = max(1, total_samples // 6)
            front_arc = msg.ranges[center - arc_samples : center + arc_samples]
            valid_ranges = [r for r in front_arc if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            if valid_ranges:
                self.front_min_dist = min(valid_ranges)
            else:
                self.front_min_dist = 10.0

    def control_loop(self):
        if self.current_wp_idx >= len(self.waypoints):
            # Mission completed
            twist = Twist()
            self.cmd_pub.publish(twist)
            status_msg = String()
            status_msg.data = "MISSION_COMPLETE"
            self.status_pub.publish(status_msg)
            return

        target_wp = self.waypoints[self.current_wp_idx]
        dx = target_wp["x"] - self.robot_x
        dy = target_wp["y"] - self.robot_y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.robot_yaw), math.cos(target_yaw - self.robot_yaw))

        twist = Twist()

        if self.state == 'NAVIGATING':
            if dist < 0.35:
                # Reached Waypoint: Perform 360 Scan
                self.state = 'SCANNING'
                self.pause_start_time = time.time()
                self.get_logger().info(f"Arrived at {target_wp['name']}. Initiating 360-degree inspection scan...")
            elif self.front_min_dist < 0.40:
                # Obstacle detected in close proximity: Steer away safely around obstacle
                self.get_logger().warn(f"Obstacle in path ({self.front_min_dist:.2f}m). Steering around obstacle...")
                twist.linear.x = 0.08
                twist.angular.z = 0.7
                self.cmd_pub.publish(twist)
            else:
                if abs(yaw_err) > 0.25:
                    twist.angular.z = 0.6 if yaw_err > 0 else -0.6
                else:
                    twist.linear.x = min(0.35, max(0.15, 0.5 * dist))
                    twist.angular.z = 0.4 * yaw_err
                self.cmd_pub.publish(twist)

        elif self.state == 'SCANNING':
            elapsed = time.time() - self.pause_start_time
            if elapsed < 6.28: # Perform 360 rotation scan
                twist.angular.z = 1.0
                self.cmd_pub.publish(twist)
            else:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info(f"Completed inspection scan at {target_wp['name']}.")
                self.current_wp_idx += 1
                self.state = 'NAVIGATING'

        # Broadcast status message
        status_str = f"STATE:{self.state} | WAYPOINT:{target_wp['name']} ({self.current_wp_idx+1}/{len(self.waypoints)}) | DIST:{dist:.2f}m"
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = InspectionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
