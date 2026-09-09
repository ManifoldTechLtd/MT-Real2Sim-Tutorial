#!/bin/bash
# ros_teleop.sh — 在 Docker 容器内同时启动 bridge + teleop keyboard
#
# 用法 (宿主机):
#   docker exec -it real2sim2real_ros1 bash /catkin_ws/src/../tools/ros_teleop.sh
#   或:
#   ./tools/ros_teleop.sh          # 自动 docker exec
#
# 流程:
#   1. 后台启动 roscore + sim_bridge (TCP 桥接)
#   2. 前台启动 teleop_keyboard (键盘遥操作)
#   3. Ctrl+C 退出 teleop 时自动清理后台进程

set -e

# 检测是否在 Docker 容器内
if [ ! -f /catkin_ws/devel/setup.bash ]; then
    echo "[ros_teleop] Not inside Docker container. Launching via docker exec..."
    exec docker exec -it real2sim2real_ros1 bash -c "source /catkin_ws/devel/setup.bash && bash /catkin_ws/src/../tools/ros_teleop.sh"
fi

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "============================================"
echo "  ROS1 Teleop: bridge + keyboard control"
echo "============================================"

# 清理函数
cleanup() {
    echo ""
    echo "[ros_teleop] Shutting down..."
    # 杀掉后台 roslaunch
    if [ -n "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null
        wait $BRIDGE_PID 2>/dev/null
    fi
    # 杀掉 roscore
    killall -q roscore rosmaster rosout 2>/dev/null || true
    echo "[ros_teleop] Done."
}
trap cleanup EXIT

# 1. 启动 roscore
echo "[ros_teleop] Starting roscore..."
roscore &
ROSCORE_PID=$!
sleep 2

# 2. 后台启动 bridge
echo "[ros_teleop] Starting sim_bridge (background)..."
roslaunch sim_bridge bridge.launch --wait &
BRIDGE_PID=$!
sleep 2

# 3. 前台启动 teleop
echo "[ros_teleop] Starting keyboard teleop (foreground)..."
echo "[ros_teleop] Press WASD/QE to control, Space=stop, Ctrl+C=quit"
echo ""
rosrun nav_demo teleop_keyboard.py \
    _linear_speed:=1.5 \
    _lateral_speed:=1.0 \
    _angular_speed:=1.5
