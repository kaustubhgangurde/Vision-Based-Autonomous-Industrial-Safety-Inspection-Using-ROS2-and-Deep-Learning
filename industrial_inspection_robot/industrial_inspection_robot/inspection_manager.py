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
        self.left_min_dist = 10.0
        self.right_min_dist = 10.0
        
        # Optimized Industrial Floor Inspection Waypoints
        # Phase 1: SLAM Mapping & Corridor Patrol
        # Phase 2: Hazard Survey & Final Safety Report
        self.waypoints = [
            {"name": "Checkpoint A: East Corridor (SLAM Mapping)", "x": 1.8, "y": 0.0},
            {"name": "Checkpoint B: North Storage (Perimeter Sweep)", "x": 1.8, "y": 2.2},
            {"name": "Checkpoint C: West Chemical Bay (Hazard Survey)", "x": -1.8, "y": 2.2},
            {"name": "Checkpoint D: South Equipment Bay (Hazard Survey)", "x": -1.8, "y": -1.8},
            {"name": "Checkpoint E: Central Hub (Final Inspection & Report)", "x": 0.0, "y": 0.0}
        ]
        
        self.current_wp_idx = 0
        self.state = 'NAVIGATING'
        self.pause_start_time = None
        
        self.avoidance_dir = 0.0  # Latched avoidance direction (1.0 for Left, -1.0 for Right)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous Inspection Route Manager active. High-Sensitivity Obstacle Avoidance enabled.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            total_samples = len(msg.ranges)
            center = total_samples // 2
            arc_samples = max(1, total_samples // 6)
            
            front_arc = msg.ranges[center - arc_samples : center + arc_samples]
            valid_front = [r for r in front_arc if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            self.front_min_dist = min(valid_front) if valid_front else 10.0

            left_arc = msg.ranges[center + arc_samples : center + 2*arc_samples]
            valid_left = [r for r in left_arc if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            self.left_min_dist = min(valid_left) if valid_left else 10.0

            right_arc = msg.ranges[center - 2*arc_samples : center - arc_samples]
            valid_right = [r for r in right_arc if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            self.right_min_dist = min(valid_right) if valid_right else 10.0

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
            if dist < 0.45:
                # Reached Waypoint: Brief pause for camera sampling (No 360-degree spins)
                self.state = 'SCANNING'
                self.avoidance_dir = 0.0
                self.pause_start_time = time.time()
                self.get_logger().info(f"Arrived at {target_wp['name']}. Performing visual inspection sweep...")
            elif self.front_min_dist < 0.65:
                # Obstacle Avoidance Mode (Latched turn direction)
                if self.avoidance_dir == 0.0:
                    self.avoidance_dir = 1.0 if self.left_min_dist >= self.right_min_dist else -1.0
                
                self.get_logger().warn(f"Obstacle ahead ({self.front_min_dist:.2f}m). Steering {'LEFT' if self.avoidance_dir > 0 else 'RIGHT'}...")
                
                if self.front_min_dist < 0.35:
                    # Too close (< 0.35m): Backup and turn
                    twist.linear.x = -0.10
                    twist.angular.z = 0.8 * self.avoidance_dir
                else:
                    # Smooth avoidance curve
                    twist.linear.x = 0.10
                    twist.angular.z = 0.7 * self.avoidance_dir
                
                self.cmd_pub.publish(twist)
            else:
                # Clear Path: Reset avoidance latch & navigate to waypoint
                self.avoidance_dir = 0.0
                if abs(yaw_err) > 0.30:
                    twist.angular.z = 0.6 if yaw_err > 0 else -0.6
                else:
                    twist.linear.x = min(0.30, max(0.12, 0.4 * dist))
                    twist.angular.z = 0.4 * yaw_err
                self.cmd_pub.publish(twist)

        elif self.state == 'SCANNING':
            elapsed = time.time() - self.pause_start_time
            if elapsed < 2.0:  # Brief 2-second pause
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f"Completed visual inspection at {target_wp['name']}.")
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
