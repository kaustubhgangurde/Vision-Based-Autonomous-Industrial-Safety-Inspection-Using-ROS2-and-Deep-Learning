#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
import time
import math
from std_msgs.msg import String

class ReportGeneratorNode(Node):
    def __init__(self):
        super().__init__('report_generator_node')
        
        self.hazards_db = []
        self.start_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.mission_complete_logged = False
        
        self.create_subscription(String, '/inspection/detected_hazards', self.hazard_callback, 10)
        self.create_subscription(String, '/inspection/status', self.status_callback, 10)
        
        self.report_timer = self.create_timer(10.0, self.generate_reports)
        self.get_logger().info('Automated Safety Inspection Report Generator active. Listening for hazard telemetry...')

    def hazard_callback(self, msg):
        try:
            detected_list = json.loads(msg.data)
            for hazard in detected_list:
                # Check for spatial deduplication (within 0.8m)
                is_duplicate = False
                for existing in self.hazards_db:
                    if existing["type"] == hazard["type"]:
                        dist = math.hypot(existing["map_x"] - hazard["map_x"], existing["map_y"] - hazard["map_y"])
                        if dist < 0.8:
                            is_duplicate = True
                            # Update confidence if higher
                            existing["confidence"] = max(existing["confidence"], hazard["confidence"])
                            break
                if not is_duplicate:
                    hazard["id"] = f"HAZ-{len(self.hazards_db)+1:03d}"
                    self.hazards_db.append(hazard)
                    self.get_logger().info(f"New Hazard Recorded: {hazard['id']} - {hazard['type']} at ({hazard['map_x']}m, {hazard['map_y']}m)")
                    self.generate_reports()
        except Exception as e:
            self.get_logger().error(f"Error parsing hazard data: {e}")

    def status_callback(self, msg):
        if "MISSION_COMPLETE" in msg.data:
            if not self.mission_complete_logged:
                self.mission_complete_logged = True
                self.get_logger().info("Mission completion received. Compiling final safety inspection report...")
                self.generate_reports()

    def generate_reports(self):
        if not self.hazards_db:
            return

        directories = [
            os.path.expanduser('~/inspection_reports'),
            os.path.expanduser('~/inspection_robot_ws/reports')
        ]
        for d in directories:
            os.makedirs(d, exist_ok=True)

        critical_cnt = sum(1 for h in self.hazards_db if h["severity"] == "CRITICAL")
        high_cnt = sum(1 for h in self.hazards_db if h["severity"] == "HIGH")
        medium_cnt = sum(1 for h in self.hazards_db if h["severity"] == "MEDIUM")

        # 1. HTML Report Generation
        html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Autonomous Industrial Safety Inspection Report</title>
    <style>
        body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #0f172a; color: #f8fafc; margin: 0; padding: 30px; }}
        .header {{ border-bottom: 2px solid #334155; padding-bottom: 20px; margin-bottom: 30px; }}
        h1 {{ color: #38bdf8; margin: 0 0 10px 0; }}
        .meta {{ color: #94a3b8; font-size: 14px; }}
        .summary-cards {{ display: flex; gap: 20px; margin-bottom: 30px; }}
        .card {{ background: #1e293b; padding: 20px; border-radius: 10px; flex: 1; border: 1px solid #334155; text-align: center; }}
        .card h2 {{ margin: 0; font-size: 32px; }}
        .card p {{ margin: 5px 0 0 0; color: #94a3b8; font-size: 14px; }}
        .critical {{ color: #ef4444; }}
        .high {{ color: #f59e0b; }}
        .medium {{ color: #3b82f6; }}
        table {{ width: 100%; border-collapse: collapse; background: #1e293b; border-radius: 10px; overflow: hidden; }}
        th, td {{ padding: 14px 18px; text-align: left; border-bottom: 1px solid #334155; }}
        th {{ background: #0f172a; color: #38bdf8; font-weight: 600; }}
        tr:hover {{ background: #334155; }}
        .badge {{ padding: 4px 10px; border-radius: 20px; font-size: 12px; font-weight: bold; display: inline-block; }}
        .badge-critical {{ background: #7f1d1d; color: #fca5a5; }}
        .badge-high {{ background: #78350f; color: #fde68a; }}
        .badge-medium {{ background: #1e3a8a; color: #bfdbfe; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>Autonomous Industrial Safety Inspection Report</h1>
        <div class="meta">Facility: Industrial Assembly Plant #4 | Generated: {time.strftime('%Y-%m-%d %H:%M:%S')} | Start: {self.start_time}</div>
    </div>

    <div class="summary-cards">
        <div class="card">
            <h2>{len(self.hazards_db)}</h2>
            <p>Total Hazards Identified</p>
        </div>
        <div class="card">
            <h2 class="critical">{critical_cnt}</h2>
            <p>Critical Hazards</p>
        </div>
        <div class="card">
            <h2 class="high">{high_cnt}</h2>
            <p>High Severity</p>
        </div>
        <div class="card">
            <h2 class="medium">{medium_cnt}</h2>
            <p>Medium Severity</p>
        </div>
    </div>

    <h2>Identified Safety Hazards & Spatial Locations</h2>
    <table>
        <thead>
            <tr>
                <th>Hazard ID</th>
                <th>Classification</th>
                <th>Severity</th>
                <th>Map Coordinates (X, Y)</th>
                <th>Confidence</th>
                <th>Timestamp</th>
            </tr>
        </thead>
        <tbody>
"""
        for h in self.hazards_db:
            badge_cls = f"badge-{h['severity'].lower()}"
            html_content += f"""
            <tr>
                <td><strong>{h['id']}</strong></td>
                <td>{h['type']}</td>
                <td><span class="badge {badge_cls}">{h['severity']}</span></td>
                <td>({h['map_x']}m, {h['map_y']}m)</td>
                <td>{int(h['confidence']*100)}%</td>
                <td>{h['timestamp']}</td>
            </tr>"""

        html_content += """
        </tbody>
    </table>
</body>
</html>
"""
        # 2. Markdown Report Generation
        md_content = f"# Autonomous Industrial Safety Inspection Report\n\n"
        md_content += f"- **Facility**: Industrial Assembly Plant #4\n"
        md_content += f"- **Inspection Timestamp**: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        md_content += f"- **Total Hazards Logged**: {len(self.hazards_db)}\n\n"
        md_content += f"| Hazard ID | Classification | Severity | Map Location (X, Y) | Confidence | Timestamp |\n"
        md_content += f"| --- | --- | --- | --- | --- | --- |\n"
        for h in self.hazards_db:
            md_content += f"| `{h['id']}` | {h['type']} | **{h['severity']}** | ({h['map_x']}m, {h['map_y']}m) | {int(h['confidence']*100)}% | {h['timestamp']} |\n"

        # 3. Write Reports to All System Locations
        for target_dir in directories:
            for html_filename in ['latest_report.html', 'inspection_report.html', 'safety_inspection_report.html']:
                with open(os.path.join(target_dir, html_filename), 'w') as f:
                    f.write(html_content)
            for md_filename in ['latest_report.md', 'safety_inspection_report.md']:
                with open(os.path.join(target_dir, md_filename), 'w') as f:
                    f.write(md_content)
            with open(os.path.join(target_dir, 'inspection_summary.json'), 'w') as f:
                json.dump({"hazards": self.hazards_db, "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=4)

def main(args=None):
    rclpy.init(args=args)
    node = ReportGeneratorNode()
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
