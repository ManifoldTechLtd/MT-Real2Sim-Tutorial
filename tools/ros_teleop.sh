#!/bin/bash
# ros_teleop.sh — Start bridge + teleop keyboard simultaneously inside Docker container
#
# Usage (host):
#   docker exec -it real2sim2real_ros1 bash /catkin_ws/src/../tools/ros_teleop.sh
#   or:
#   ./tools/ros_teleop.sh          # Auto docker exec
#
# Flow:
#   1. Start roscore + sim_bridge in background (TCP bridge)
#   2. Start teleop_keyboard in foreground (keyboard teleoperation)
#   3. Auto-cleanup background processes on Ctrl+C

set -e

# Detect if running inside Docker container
if [ ! -f /catkin_ws/devel/setup.bash ]; then
    # Prefer docker-full container name, fall back to legacy
    if docker ps --format '{{.Names}}' | grep -q '^r2s2r_ros1$'; then
        CONTAINER=r2s2r_ros1
    else
        CONTAINER=real2sim2real_ros1
    fi
    echo "[ros_teleop] Not inside Docker container. Launching via docker exec ($CONTAINER)..."
    exec docker exec -it "$CONTAINER" bash -c "source /catkin_ws/devel/setup.bash && bash /tools/ros_teleop.sh"
fi

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "============================================"
echo "  ROS1 Teleop: bridge + keyboard control"
echo "============================================"

# Cleanup function
cleanup() {
    echo ""
    echo "[ros_teleop] Shutting down..."
    # Kill background roslaunch
    if [ -n "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null
        wait $BRIDGE_PID 2>/dev/null
    fi
    # Kill roscore
    killall -q roscore rosmaster rosout 2>/dev/null || true
    echo "[ros_teleop] Done."
}
trap cleanup EXIT

# 1. Start roscore
echo "[ros_teleop] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 2

# 2. Start bridge in background
echo "[ros_teleop] Starting sim_bridge (background)..."
roslaunch sim_bridge bridge.launch --wait &
BRIDGE_PID=$!
sleep 2

# 3. Start teleop in foreground
echo "[ros_teleop] Starting keyboard teleop (foreground)..."
echo "[ros_teleop] Press WASD/QE to control, R=reset, Space=stop, Ctrl+C=quit"
echo ""
rosrun nav_demo teleop_keyboard.py \
    _linear_speed:=3.0 \
    _lateral_speed:=2.0 \
    _angular_speed:=3.0
