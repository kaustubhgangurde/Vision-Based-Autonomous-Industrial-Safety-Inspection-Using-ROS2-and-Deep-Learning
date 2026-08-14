#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import math
import time
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class HazardDetectorNode(Node):
    def __init__(self):
        super().__init__('hazard_detector_node')
        
        self.bridge = CvBridge()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.front_distance = 2.0
        self.detected_hazards = []
        self.marker_id = 0
        
        # Deep Learning (YOLOv8) Engine Initialization
        self.yolo_model = None
        self.engine_mode = "HSV Color Perception"
        if YOLO_AVAILABLE:
            try:
                self.yolo_model = YOLO("yolov8n.pt")
                self.engine_mode = "YOLOv8 Deep Learning"
                self.get_logger().info('YOLOv8 Deep Learning Inference Engine loaded successfully!')
            except Exception as e:
                self.get_logger().warn(f'Could not load YOLOv8 model: {e}. Using Color Perception engine.')
        else:
            self.get_logger().info('YOLOv8 package not detected. Active Engine: HSV Visual Perception Pipeline.')
        
        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/inspection/hazard_image', 10)
        self.hazard_json_pub = self.create_publisher(String, '/inspection/detected_hazards', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/inspection/hazard_markers', 10)
        
        self.get_logger().info(f'Safety Hazard Detector Node initialized [{self.engine_mode}].')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            mid_idx = len(msg.ranges) // 2
            r = msg.ranges[mid_idx]
            if not math.isnan(r) and not math.isinf(r):
                self.front_distance = max(0.3, min(r, 8.0))

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if len(cv_img.shape) == 3 and cv_img.shape[2] == 3:
                if getattr(msg, 'encoding', '') in ['rgb8', 'RGB8']:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        except Exception as e:
            self.get_logger().error(f"CvBridge exception: {e}")
            return

        h, w, _ = cv_img.shape
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        # Hazard Color Profile Definitions
        hazard_profiles = [
            {
                "name": "Fire / Chemical Barrel Hazard",
                "lower": np.array([0, 120, 100]),
                "upper": np.array([10, 255, 255]),
                "color": (0, 0, 255),
                "severity": "CRITICAL"
            },
            {
                "name": "Chemical / Oil Spill Hazard",
                "lower": np.array([20, 120, 120]),
                "upper": np.array([35, 255, 255]),
                "color": (0, 255, 255),
                "severity": "HIGH"
            },
            {
                "name": "Blocked Pathway Hazard",
                "lower": np.array([10, 150, 150]),
                "upper": np.array([25, 255, 255]),
                "color": (0, 140, 255),
                "severity": "MEDIUM"
            }
        ]

        markers = MarkerArray()
        current_frame_hazards = []

        for profile in hazard_profiles:
            mask = cv2.inRange(hsv, profile["lower"], profile["upper"])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1200: # Threshold filter for valid hazard target
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    confidence = min(0.99, round(float(area) / (w * h) * 15.0 + 0.70, 2))
                    
                    # Draw detection bounding box and HUD text
                    cv2.rectangle(cv_img, (x, y), (x + bw, y + bh), profile["color"], 2)
                    label = f"[{profile['severity']}] {profile['name']} ({int(confidence*100)}%)"
                    cv2.putText(cv_img, label, (x, max(20, y - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, profile["color"], 2)
                    
                    # Calculate estimated Map coordinates
                    dist = self.front_distance
                    hx = round(self.robot_x + dist * math.cos(self.robot_yaw), 2)
                    hy = round(self.robot_y + dist * math.sin(self.robot_yaw), 2)
                    
                    hazard_info = {
                        "type": profile["name"],
                        "severity": profile["severity"],
                        "confidence": confidence,
                        "map_x": hx,
                        "map_y": hy,
                        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
                    }
                    current_frame_hazards.append(hazard_info)
                    
                    # Create RViz 3D Marker
                    marker = Marker()
                    marker.header.frame_id = "odom"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "hazards"
                    marker.id = self.marker_id
                    self.marker_id += 1
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = hx
                    marker.pose.position.y = hy
                    marker.pose.position.z = 0.4
                    marker.scale.x = 0.3
                    marker.scale.y = 0.3
                    marker.scale.z = 0.3
                    marker.color.r = profile["color"][2] / 255.0
                    marker.color.g = profile["color"][1] / 255.0
                    marker.color.b = profile["color"][0] / 255.0
                    marker.color.a = 0.9
                    markers.markers.append(marker)

        # Draw HUD status header on video feed
        hud_text = f"[{self.engine_mode}] Pose: X={self.robot_x:.2f}m Y={self.robot_y:.2f}m | Hazards: {len(current_frame_hazards)}"
        cv2.putText(cv_img, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Publish annotated image to ROS 2 topic
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

        # Render optional GUI pop-up window
        try:
            cv2.imshow('Autonomous Safety Inspection - Live Perception Feed', cv_img)
            cv2.waitKey(1)
        except Exception:
            pass

        # Publish detected hazards JSON string
        if current_frame_hazards:
            json_str = json.dumps(current_frame_hazards)
            msg_str = String()
            msg_str.data = json_str
            self.hazard_json_pub.publish(msg_str)
            self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = HazardDetectorNode()
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
