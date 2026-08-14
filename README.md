# Vision-Based Autonomous Industrial Safety Inspection Using ROS 2 and Deep Learning

An autonomous industrial inspection robot simulation framework built on ROS 2 Jazzy / Humble, Gazebo Sim, and Computer Vision / Deep Learning. The system deploys an autonomous mobile robot (AMR) equipped with LiDAR, Differential Drive, and RGB Camera sensors into an industrial facility to navigate inspection routes, detect safety hazards (fire, chemical spills, pathway blockages), build real-time 2D occupancy maps, and produce executive safety inspection reports.

---

## Key Features

- **Autonomous Inspection Navigation**: LiDAR-based obstacle detection and autonomous wayward navigation across industrial checkpoints.
- **Computer Vision & Hazard Detection**: Visual perception pipeline supporting HSV color segmentation and YOLOv8 deep learning models for detecting safety hazards in real-time.
- **SLAM & Occupancy Mapping**: `slam_toolbox` integration for real-time 2D map construction.
- **RViz2 Integration**: Live dashboard displaying 2D maps, robot URDF model transforms, 2D hazard position markers, and live camera feed.
- **Executive Report Generation**: Automated HTML and JSON report compilation upon completing inspection sweeps (`~/inspection_reports/`).
- **One-Command Master Launch**: Automated execution script (`run_demo.sh`) for single-terminal build, sourcing, and simulation deployment.

---

## Package Architecture

```
industrial_inspection_robot/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── burger_world.sdf            # Gazebo 3D Industrial Environment
│   ├── inspection.rviz              # Pre-configured RViz2 Visual Dashboard
│   ├── mapper_params_online_async.yaml # SLAM Toolbox Configuration
│   └── nav2_params.yaml            # Navigation Stack Parameters
├── description/
│   ├── robot.urdf.xacro            # Robot Top-Level URDF/Xacro Assembly
│   ├── gazebo_control.xacro        # Gazebo DiffDrive & Joint State Plugins
│   ├── lidar.xacro                 # 2D LiDAR Ray Sensor Definition
│   └── camera.xacro                # RGB Camera Sensor Definition
├── industrial_inspection_robot/
│   ├── data_acq.py                 # Telemetry & Sensor Data Sampler Node
│   ├── hazard_detector.py          # CV & YOLOv8 Hazard Detection Pipeline
│   ├── inspection_manager.py       # Autonomous Route & Obstacle Avoidance Controller
│   └── report_generator.py        # Executive HTML/JSON Report Compiler
└── launch/
    ├── rsp.launch.py               # Robot State Publisher & Joint State Publisher
    ├── sim.launch.py               # Gazebo Sim & Parameter Bridge Launch
    └── navigation.launch.py        # Master Simulation Orchestrator
```

---

## Quick Start & Installation

### Prerequisites

- **ROS 2**: Jazzy, Humble, or Iron
- **Gazebo Sim**: `ros-gz-sim` and `ros-gz-bridge`
- **Dependencies**: `slam_toolbox`, `robot_state_publisher`, `joint_state_publisher`, `cv_bridge`
- **Python Packages**: `opencv-python`, `numpy`, `ultralytics` (optional for YOLOv8)

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-ros-gz-sim ros-$ROS_DISTRO-ros-gz-bridge ros-$ROS_DISTRO-slam-toolbox
pip install opencv-python numpy ultralytics
```

### Running the Full Master Simulation

Run the automated single-command launch script:

```bash
cd ~/inspection_robot_ws
bash run_demo.sh
```

---

## System Topics & Interfaces

| Topic Name | Message Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity drive control commands |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry position estimates |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D LiDAR distance range measurements |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Live RGB camera sensor stream |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Real-time SLAM 2D floor map |
| `/inspection/hazard_image` | `sensor_msgs/msg/Image` | Annotated vision stream with hazard bounding boxes |
| `/inspection/hazard_markers` | `visualization_msgs/msg/MarkerArray` | 3D hazard location spheres in RViz |
| `/inspection/status` | `std_msgs/msg/String` | State machine telemetry & route progress |

---

## License

MIT License. Designed and developed for autonomous industrial safety research.
