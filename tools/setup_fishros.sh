#!/bin/bash
# 切换到 Fishros Docker 配置
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================"
echo "  切换到 Fishros Docker 配置"
echo "============================================"
echo ""

cd "$PROJECT_ROOT/docker"

# 停止旧容器
echo "[1/4] 停止旧容器..."
docker compose down 2>/dev/null || true

# 使用 fishros 配置
echo "[2/4] 切换到 fishros 配置..."
ln -sf docker-compose.fishros.yml docker-compose.yml

# 构建新镜像
echo "[3/4] 构建 fishros 镜像 (可能需要 10-15 分钟)..."
docker compose build

# 启动容器
echo "[4/4] 启动容器..."
docker compose up -d

echo ""
echo "============================================"
echo "  Fishros Docker 配置完成!"
echo "============================================"
echo ""
echo "使用方法:"
echo "  1. 编译工作空间:"
echo "     docker exec real2sim2real_ros1 bash -c 'cd /catkin_ws && catkin_make'"
echo ""
echo "  2. 启动 teleop:"
echo "     docker exec -it real2sim2real_ros1 bash -c 'source /catkin_ws/devel/setup.bash && bash /tools/ros_teleop.sh'"
echo ""
echo "  3. 启动 RViz:"
echo "     xhost +local:docker"
echo "     docker exec -it real2sim2real_ros1 bash -c 'source /catkin_ws/devel/setup.bash && rosrun rviz rviz'"
echo ""
