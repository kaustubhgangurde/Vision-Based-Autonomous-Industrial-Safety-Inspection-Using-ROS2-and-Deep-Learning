#!/bin/bash
set -e

echo "=========================================================="
echo "   Autonomous Industrial Safety Inspection Robot Launch   "
echo "=========================================================="

WS_DIR="/home/kaustubh/inspection_robot_ws"
SRC_DIR="$WS_DIR/src/Vision-Based-Autonomous-Industrial-Safety-Inspection-Using-ROS2-and-Deep-Learning/industrial_inspection_robot"

cd "$WS_DIR"

echo "[1/4] Setting executable permissions on Python nodes..."
chmod +x "$SRC_DIR/industrial_inspection_robot/"*.py 2>/dev/null || true

echo "[2/4] Cleaning build artifacts to prevent stale cache..."
rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"

echo "[3/4] Building ROS 2 package with colcon..."
colcon build

echo "[4/4] Sourcing environment and launching master simulation..."
source "$WS_DIR/install/setup.bash"

ros2 launch industrial_inspection_robot navigation.launch.py
