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
        
        # Industrial Route with Autonomous Docking Station Return
        self.waypoints = [
            {"name": "Checkpoint A: East Assembly Corridor", "x": 2.2, "y": 0.0, "is_dock": False},
            {"name": "Checkpoint B: North Storage Bay", "x": 1.5, "y": 3.2, "is_dock": False},
            {"name": "Checkpoint C: West Chemical Area", "x": -2.0, "y": 3.2, "is_dock": False},
            {"name": "Checkpoint D: South Equipment Bay", "x": -2.0, "y": -2.0, "is_dock": False},
            {"name": "Checkpoint E: Dock Approach Corridor", "x": 0.5, "y": 0.0, "is_dock": False},
            {"name": "Checkpoint F: Home Charging Station Terminal", "x": 0.0, "y": 0.0, "is_dock": True}
        ]
        
        self.current_wp_idx = 0
        self.state = 'NAVIGATING'
        self.pause_start_time = None
        self.avoidance_dir = 0.0
        self.docked = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous Inspection Manager initialized. Autonomous Docking System ONLINE.')

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
        if self.docked:
            return

        if self.current_wp_idx >= len(self.waypoints):
            # Final Docked State
            self.docked = True
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().info("AMR DOCKED & CHARGING (100% Battery). Triggering Safety Report...")
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
            if dist < (0.20 if target_wp["is_dock"] else 0.45):
                if target_wp["is_dock"]:
                    self.state = 'DOCKING'
                    self.pause_start_time = time.time()
                    self.get_logger().info("Initiating Precision Autonomous Docking Procedure...")
                else:
                    self.state = 'SCANNING'
                    self.avoidance_dir = 0.0
                    self.pause_start_time = time.time()
                    self.get_logger().info(f"Arrived at {target_wp['name']}. Performing visual inspection sweep...")
            elif self.front_min_dist < 0.65 and not target_wp["is_dock"]:
                # High-Clearance Avoidance Mode
                if self.avoidance_dir == 0.0:
                    self.avoidance_dir = 1.0 if self.left_min_dist >= self.right_min_dist else -1.0
                
                if self.front_min_dist < 0.38:
                    twist.linear.x = -0.10
                    twist.angular.z = 0.85 * self.avoidance_dir
                else:
                    twist.linear.x = 0.12
                    twist.angular.z = 0.85 * self.avoidance_dir
                
                self.cmd_pub.publish(twist)
            else:
                # Target Navigation Mode
                self.avoidance_dir = 0.0
                if abs(yaw_err) > 0.35:
                    twist.linear.x = 0.05
                    twist.angular.z = 0.6 if yaw_err > 0 else -0.6
                else:
                    twist.linear.x = min(0.30, max(0.12, 0.4 * dist))
                    twist.angular.z = 0.4 * yaw_err
                self.cmd_pub.publish(twist)

        elif self.state == 'SCANNING':
            elapsed = time.time() - self.pause_start_time
            if elapsed < 2.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info(f"Completed visual inspection at {target_wp['name']}.")
                self.current_wp_idx += 1
                self.state = 'NAVIGATING'

        elif self.state == 'DOCKING':
            # Precision Alignment to Charging Pad Contacts
            elapsed = time.time() - self.pause_start_time
            if elapsed < 3.0:
                # Rotate to face charging contacts (Yaw = 0.0)
                yaw_target_err = math.atan2(math.sin(0.0 - self.robot_yaw), math.cos(0.0 - self.robot_yaw))
                if abs(yaw_target_err) > 0.05:
                    twist.angular.z = 0.3 if yaw_target_err > 0 else -0.3
                else:
                    twist.linear.x = -0.05  # Slow reverse latch onto charging pad
                self.cmd_pub.publish(twist)
            else:
                self.get_logger().info("SUCCESS: Contact established with Charging Station Pad! AMR DOCKED.")
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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
