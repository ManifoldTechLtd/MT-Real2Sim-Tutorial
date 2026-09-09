#!/bin/bash
# 测试点云频率 - 不启动teleop

set -e

COMPOSE_FILE="docker/docker-compose.yml"
ROS_CONTAINER="isaac_nav_ros1"
ISAAC_CONTAINER="isaac_nav_isaac"

echo "=========================================="
echo "点云频率测试"
echo "=========================================="
echo ""

# 启动容器
echo "启动容器..."
ROBOT_TYPE="${1:-go2}" \
DATASET_DIR="/home/mt/dzy/tunnel-inspection-v2/dataset" \
docker compose -f "$COMPOSE_FILE" up -d

# 等待 Isaac Sim
echo "等待 Isaac Sim 启动..."
sleep 45

# 等待 TCP 端口
echo "等待 TCP:9090..."
WAITED=0
while ! ss -tlnp 2>/dev/null | grep -q ":9090 " && [ "$WAITED" -lt 60 ]; do
    sleep 3
    WAITED=$((WAITED + 3))
done

echo "✓ Isaac Sim 就绪"

# 启动 roscore
echo "启动 roscore..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roscore >/tmp/roscore.log 2>&1"
sleep 3

# 启动 bridge
echo "启动 sim_bridge..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roslaunch sim_bridge bridge.launch --wait >/tmp/bridge.log 2>&1"
sleep 5

echo ""
echo "=========================================="
echo "测量点云频率（30秒）"
echo "=========================================="

docker exec "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash && timeout 30 rostopic hz /registered_scan 2>&1" || true

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "停止容器: docker compose -f $COMPOSE_FILE down"
