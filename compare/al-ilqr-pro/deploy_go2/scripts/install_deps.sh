#!/bin/bash
# AL-iLQR Go2 依赖安装脚本
# 适用于 Ubuntu 20.04 + ROS Noetic (Jetson NX)

set -e

echo "=========================================="
echo " AL-iLQR Go2 依赖安装"
echo "=========================================="

# 检查 ROS
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS 未初始化，请先 source /opt/ros/noetic/setup.bash"
    exit 1
fi

if [ "$ROS_DISTRO" != "noetic" ]; then
    echo "[WARN] 检测到 ROS $ROS_DISTRO，推荐使用 noetic"
fi

echo "[INFO] 安装系统依赖..."
sudo apt-get update
sudo apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    build-essential \
    cmake

echo ""
echo "=========================================="
echo " 依赖安装完成！"
echo " 下一步：cd catkin_ws && catkin build al_ilqr_go2"
echo "=========================================="
