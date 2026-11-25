#!/bin/bash
# start.sh — Isaac Navigation Sim 一键启动脚本
#
# 用法:
#   ./start.sh                          # 启动 (默认 kinematic_car)
#   ./start.sh --robot spot             # 启动 spot
#   ./start.sh --robot go2              # 启动 go2
#   ./start.sh --robot kinematic_car    # 显式指定 kinematic_car
#   ./start.sh --scene /path/to/scene   # 指定场景目录
#   ./start.sh --stop                   # 停止所有容器
#   ./start.sh --help                   # 帮助
#
# 流程:
#   1. Pre-flight 检查 (Docker, GPU, 镜像, 场景)
#   2. 启动双容器 (Isaac Sim + ROS1)
#   3. 等待 Isaac Sim 就绪 (TCP:9090)
#   4. 启动 ROS 节点 (roscore + bridge + RViz + teleop)
#   5. Ctrl+C 自动清理

set -e

# ── 项目配置 ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
COMPOSE_FILE="$PROJECT_DIR/docker/docker-compose.yml"
ISAAC_CONTAINER="isaac_nav_isaac"
ROS_CONTAINER="isaac_nav_ros1"
DATASET_DIR="${DATASET_DIR:-$PROJECT_DIR/dataset}"
SIM_CONFIG="${SIM_CONFIG:-}"
TCP_PORT=9090
MAX_WAIT="${MAX_WAIT:-600}"
LOG_DIR="${LOG_DIR:-/tmp/isaac_nav_sim}"
mkdir -p "$LOG_DIR"

# ── 颜色输出 ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${CYAN}[start]${NC} $*"; }
ok()    { echo -e "${GREEN}[start]${NC} $*"; }
warn()  { echo -e "${YELLOW}[start]${NC} $*"; }
err()   { echo -e "${RED}[start] ERROR:${NC} $*" >&2; }

# ── 参数解析 ──
ACTION="start"
ROBOT_TYPE=""
SCENE_PATH=""
RUNTIME=""
STACK="none"
MAP_NAME="office_0923_sim"
ONS2_ROOT="${ONS2_ROOT:-$PROJECT_DIR/../Odin-Nav-Stack2}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stop)
            ACTION="stop"
            shift
            ;;
        --robot)
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --runtime)
            RUNTIME="$2"
            shift 2
            ;;
        --stack)
            STACK="$2"
            shift 2
            ;;
        --map)
            MAP_NAME="$2"
            shift 2
            ;;
        --ons2-root)
            ONS2_ROOT="$2"
            shift 2
            ;;
        --scene)
            DATASET_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "Isaac Navigation Sim - 一键启动脚本"
            echo ""
            echo "用法: ./start.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --robot <type>    指定机器人类型: kinematic_car | spot | go2"
            echo "                    (默认: kinematic_car, 或使用 config.yaml 中的配置)"
            echo "  --scene <dir>     指定场景文件目录"
            echo "  --runtime <r>     ros1 | ros2（--stack ons2 时必填）"
            echo "  --stack <s>       none | ons2（默认 none，保持原有流程）"
            echo "  --map <name>      ONS2 地图名（默认 office_0923_sim）"
            echo "  --ons2-root <p>   ONS2 仓库路径（默认 ../Odin-Nav-Stack2）"
            echo "  --stop            停止所有容器"
            echo "  -h, --help        显示此帮助"
            echo ""
            echo "示例:"
            echo "  ./start.sh                           # 默认启动"
            echo "  ./start.sh --robot spot              # 启动 Spot 机器人"
            echo "  ./start.sh --robot kinematic_car     # 启动运动学履带车"
            echo "  ./start.sh --stop                    # 停止"
            exit 0
            ;;
        *)
            err "未知参数: $1"
            echo "运行 './start.sh --help' 查看帮助"
            exit 1
            ;;
    esac
done

# ── 校验运行时/栈参数 ──
if [ "$STACK" != "none" ] && [ "$STACK" != "ons2" ]; then
    err "无效的 stack: $STACK（支持 none | ons2）"
    exit 1
fi
if [ "$STACK" = "ons2" ] && [ -z "$ROBOT_TYPE" ]; then
    ROBOT_TYPE=go2
fi
if [ -z "$SIM_CONFIG" ]; then
    if [ "$STACK" = "ons2" ] && [ "$ROBOT_TYPE" = "go2" ]; then
        SIM_CONFIG="r2s2r/configs/go2_office_0923.yaml"
    else
        SIM_CONFIG="r2s2r/configs/config.yaml"
    fi
fi
if [ "$STACK" = "ons2" ]; then
    if [[ "$RUNTIME" != "ros1" && "$RUNTIME" != "ros2" ]]; then
        err "--stack ons2 需要 --runtime ros1|ros2"
        exit 1
    fi
    ONS2_ROOT="$(cd "$ONS2_ROOT" 2>/dev/null && pwd)" || {
        err "ONS2_ROOT 不存在: $ONS2_ROOT"
        err "先克隆 Odin-Nav-Stack2，或传 --ons2-root <path>"
        exit 1
    }
    MAP_YAML_HOST=""
    if [ ! -f "$PROJECT_DIR/integrations/ons2/maps/${MAP_NAME}.yaml" ]; then
        err "缺少地图: $PROJECT_DIR/integrations/ons2/maps/${MAP_NAME}.yaml"
        exit 1
    fi
fi

# ── 停止 ──
if [ "$ACTION" = "stop" ]; then
    info "停止容器..."
    if ! docker info &>/dev/null; then
        err "无法访问 Docker (未运行或当前用户缺少权限)"
        err "  验证: systemctl is-active docker; groups | grep -w docker"
        exit 1
    fi
    docker compose -f "$COMPOSE_FILE" down
    ok "已停止"
    exit 0
fi

# ══════════════════════════════════════
#  Pre-flight 检查
# ══════════════════════════════════════

info "运行 Pre-flight 检查..."

# Docker
if ! command -v docker &>/dev/null; then
    err "未找到 Docker. 安装: sudo apt-get install docker.io docker-compose-v2"
    exit 1
fi

if ! docker info &>/dev/null; then
    if ! systemctl is-active --quiet docker 2>/dev/null; then
        err "Docker 服务未运行. 运行: sudo systemctl start docker"
    elif ! groups | grep -qw docker; then
        err "当前用户不在 docker 组中. 运行:"
        err "  sudo usermod -aG docker \$USER"
        err "  然后注销并重新登录, 或运行: newgrp docker"
    else
        err "无法访问 Docker. 运行 'docker info' 查看详情"
    fi
    exit 1
fi

# GPU
if ! command -v nvidia-smi &>/dev/null; then
    err "未找到 nvidia-smi. 安装 NVIDIA 驱动 (>= 560)"
    exit 1
fi

if ! nvidia-smi &>/dev/null; then
    err "GPU 驱动错误. 运行 nvidia-smi 检查"
    exit 1
fi

# nvidia-container-toolkit
if ! docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi &>/dev/null 2>&1; then
    warn "Docker GPU 透传测试失败. 确保已安装 nvidia-container-toolkit:"
    warn "  sudo apt-get install nvidia-container-toolkit"
    warn "  sudo nvidia-ctk runtime configure --runtime=docker"
    warn "  sudo systemctl restart docker"
fi

# Isaac Sim 镜像
if ! docker images | grep -qE "r2s2r-isaac|isaacsim51-r2s2r"; then
    err "未找到 Isaac Sim 镜像: r2s2r-isaac:5.1.0（或 isaacsim51-r2s2r:latest）"
    err "请按 docs/end-to-end-scan-to-ons2.md 或 docker/Dockerfile.isaac 构建/加载镜像"
    exit 1
fi

# 场景文件
if [ ! -d "$DATASET_DIR" ]; then
    warn "场景目录不存在: $DATASET_DIR"
    warn "仿真将使用默认空白地面. 请从项目文档获取场景文件"
    echo ""
fi

# 验证机器人类型
if [ -n "$ROBOT_TYPE" ]; then
    if [[ ! "$ROBOT_TYPE" =~ ^(kinematic_car|spot|go2)$ ]]; then
        err "无效的机器人类型: $ROBOT_TYPE"
        err "支持的类型: kinematic_car, spot, go2"
        exit 1
    fi
    info "机器人类型: $ROBOT_TYPE (通过参数指定)"
else
    info "机器人类型: 使用 config.yaml 中的配置"
fi

ok "Pre-flight 检查通过"

# ══════════════════════════════════════
#  清理函数
# ══════════════════════════════════════

cleanup() {
    echo ""
    info "正在关闭..."

    # 停止 ROS 进程
    docker exec "$ROS_CONTAINER" bash -c \
        "killall -q rviz roscore rosmaster rosout roslaunch 2>/dev/null; true" 2>/dev/null || true

    # 停止容器
    docker compose -f "$COMPOSE_FILE" down 2>/dev/null || true

    ok "已停止. 再见!"
}
trap cleanup EXIT

# ══════════════════════════════════════
#  启动容器
# ══════════════════════════════════════

info "启动容器 (Isaac Sim + ROS1)..."
xhost +local:docker >/dev/null 2>&1 || true

# 设置环境变量
export DATASET_DIR
export ROBOT_TYPE
export SCENE_PATH
export DISPLAY
export SIM_CONFIG
export ONS2_ROOT
export ROS_DOMAIN_ID
export MAP_NAME
export RUNTIME

if [ "$STACK" = "ons2" ] && [ "$RUNTIME" = "ros2" ]; then
    info "启动容器 (Isaac Sim + ROS2)..."
    if ! docker images | grep -q "isaac_nav_ros2"; then
        info "构建 ROS2 镜像 isaac_nav_ros2:jazzy ..."
        DATASET_DIR="$DATASET_DIR" ROBOT_TYPE="$ROBOT_TYPE" SCENE_PATH="$SCENE_PATH" \
        SIM_CONFIG="$SIM_CONFIG" ONS2_ROOT="$ONS2_ROOT" \
            docker compose -f "$COMPOSE_FILE" build ros2 || exit 1
    fi
    DATASET_DIR="$DATASET_DIR" ROBOT_TYPE="$ROBOT_TYPE" SCENE_PATH="$SCENE_PATH" \
    SIM_CONFIG="$SIM_CONFIG" ONS2_ROOT="$ONS2_ROOT" \
        docker compose -f "$COMPOSE_FILE" up -d --force-recreate isaac-sim ros2
else
    info "启动容器 (Isaac Sim + ROS1)..."
    DATASET_DIR="$DATASET_DIR" ROBOT_TYPE="$ROBOT_TYPE" SCENE_PATH="$SCENE_PATH" \
    SIM_CONFIG="$SIM_CONFIG" ONS2_ROOT="$ONS2_ROOT" \
        docker compose -f "$COMPOSE_FILE" up -d --force-recreate isaac-sim ros1
fi

# ══════════════════════════════════════
#  等待 Isaac Sim 就绪
# ══════════════════════════════════════

info "等待 Isaac Sim 启动 (监听 TCP:$TCP_PORT, 超时 ${MAX_WAIT}s)..."

# 检测是否冷启动
CACHE_NONEMPTY=$(docker exec "$ISAAC_CONTAINER" \
    bash -c 'ls /isaac-sim/kit/cache 2>/dev/null | head -1' 2>/dev/null || true)
if [ -z "$CACHE_NONEMPTY" ]; then
    warn "首次冷启动: 正在编译 Vulkan shaders, 通常需要 5-10 分钟. 请耐心等待, 不要 Ctrl+C"
    warn "如果默认 ${MAX_WAIT}s 超时不够: MAX_WAIT=1800 ./start.sh"
else
    info "发现 shader 缓存, 预计 30-60 秒就绪"
fi

WAITED=0
while ! ss -tlnp 2>/dev/null | grep -q ":${TCP_PORT} " && [ "$WAITED" -lt "$MAX_WAIT" ]; do
    sleep 3
    WAITED=$((WAITED + 3))
    if [ $((WAITED % 15)) -eq 0 ]; then
        info "已等待 ${WAITED}s / ${MAX_WAIT}s ..."
    fi
done

if [ "$WAITED" -ge "$MAX_WAIT" ]; then
    err "等待 Isaac Sim 超时 (${MAX_WAIT}s). 检查:"
    err "  docker logs $ISAAC_CONTAINER"
    err "如果首次启动仍在拉取镜像, 增加超时: MAX_WAIT=1800 ./start.sh"
    exit 1
fi

ok "Isaac Sim TCP 端口已开放 ($TCP_PORT)"

# 等待 Isaac Sim 仿真循环运行
info "等待 Isaac Sim 初始化 (检查容器日志)..."
info ">>> 当 Isaac Sim 窗口出现时, 点击 PLAY 按钮 <<<"
DATA_WAIT=0
DATA_MAX_WAIT=300
while [ "$DATA_WAIT" -lt "$DATA_MAX_WAIT" ]; do
    # 检查 Isaac 容器日志中的诊断输出
    if docker logs "$ISAAC_CONTAINER" 2>&1 | grep -q "\[diag\]\|\[run_sim\].*TCP"; then
        break
    fi
    sleep 3
    DATA_WAIT=$((DATA_WAIT + 3))
    if [ $((DATA_WAIT % 15)) -eq 0 ]; then
        LAST_LINE=$(docker logs "$ISAAC_CONTAINER" 2>&1 | grep -E "\[run_sim\]|\[diag\]|\[config\]" | tail -1)
        info "等待 ${DATA_WAIT}s... 最新 Isaac 日志: ${LAST_LINE:-<初始化中>}"
    fi
done

if [ "$DATA_WAIT" -ge "$DATA_MAX_WAIT" ]; then
    warn "等待 Isaac Sim 进入主循环超时 (${DATA_MAX_WAIT}s)"
    warn "继续执行 — bridge 会在就绪时自动重连"
fi

ok "Isaac Sim 仿真循环正在运行"

# ══════════════════════════════════════
#  ONS2 模式 (ROS2)：bridge + 导航栈
# ══════════════════════════════════════
if [ "$STACK" = "ons2" ] && [ "$RUNTIME" = "ros2" ]; then
    if ! docker exec isaac_nav_ros2 test -f /ons2/ros2_ws/install/setup.bash; then
        err "ONS2 ROS2 工作区尚未编译: /ons2/ros2_ws/install 不存在"
        err "先在容器内执行 colcon build（见 docs/simulation/04_simulation.md）"
        exit 1
    fi

    info "启动 ROS2 sim_bridge ..."
    docker exec -d isaac_nav_ros2 bash -lc \
        "source /opt/ros/jazzy/setup.bash && \
         source /ons2/ros2_ws/install/setup.bash && \
         ros2 launch sim_bridge_ros2 bridge.launch.py \
           bridge_config:=/ons2/ros2_ws/src/sim_bridge_ros2/config/go2_sim.yaml \
           color_lut_path:=/integrations/ons2/color_luts/${MAP_NAME}.npz \
           >/tmp/bridge.log 2>&1"

    ODOM_WAIT=0
    while [ "$ODOM_WAIT" -lt 60 ]; do
        if docker exec isaac_nav_ros2 bash -lc \
            "source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash && \
             timeout 3 ros2 topic echo /odin1/odometry --once >/dev/null 2>&1"; then
            break
        fi
        sleep 2
        ODOM_WAIT=$((ODOM_WAIT + 2))
    done
    if [ "$ODOM_WAIT" -ge 60 ]; then
        err "等待 /odin1/odometry 超时，查看: docker exec isaac_nav_ros2 cat /tmp/bridge.log"
        exit 1
    fi
    ok "sim_bridge 已发布 /odin1/odometry (${ODOM_WAIT}s)"

    info "启动 ONS2 导航栈 (ROS2, map=${MAP_NAME}) ..."
    docker exec -d isaac_nav_ros2 bash -lc \
        "bash /ons2/scripts/ons2_sim_ros2.sh \
           --map /integrations/ons2/maps/${MAP_NAME}.yaml \
           >/tmp/ons2_sim.log 2>&1"

    echo ""
    echo "============================================"
    echo "  ONS2 仿真栈启动中"
    echo "============================================"
    echo "  地图: ${MAP_NAME}"
    echo "  日志: docker exec isaac_nav_ros2 tail -f /tmp/ons2_sim.log"
    echo "  状态: docker exec isaac_nav_ros2 bash -lc 'source /opt/ros/jazzy/setup.bash && source /ons2/ros2_ws/install/setup.bash && ros2 topic echo /waypoint_manager/status --once'"
    echo "  航点: bash integrations/ons2/waypoints/send_waypoints.sh --runtime ros2 --scene ${MAP_NAME}"
    echo "  Ctrl+C 停止全部"
    echo "============================================"
    echo ""

    while true; do sleep 5; done
fi

# ══════════════════════════════════════
#  ROS: bridge + RViz + teleop
# ══════════════════════════════════════

info "启动 roscore..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roscore >/tmp/roscore.log 2>&1"

# 等待 roscore 就绪
ROSCORE_WAIT=0
while [ "$ROSCORE_WAIT" -lt 30 ]; do
    if docker exec "$ROS_CONTAINER" python3 -c \
        "import socket; s=socket.socket(); s.settimeout(1); s.connect(('localhost',11311)); s.close()" 2>/dev/null; then
        break
    fi
    sleep 1
    ROSCORE_WAIT=$((ROSCORE_WAIT + 1))
done
if [ "$ROSCORE_WAIT" -ge 30 ]; then
    err "roscore 启动失败 (30s 内). 检查: docker exec $ROS_CONTAINER cat /tmp/roscore.log"
    exit 1
fi
ok "roscore 就绪 (${ROSCORE_WAIT}s)"

TF_PROFILE=odin1
if [ "$ROBOT_TYPE" = "go2" ]; then
    TF_PROFILE=odin1_go2
fi
info "启动 sim_bridge (tf_profile=$TF_PROFILE, 日志: $LOG_DIR/bridge.log)..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roslaunch sim_bridge bridge.launch tf_profile:=$TF_PROFILE --wait >/tmp/bridge.log 2>&1"
sleep 3
docker cp "$ROS_CONTAINER":/tmp/roscore.log "$LOG_DIR/roscore.log" 2>/dev/null || true
docker cp "$ROS_CONTAINER":/tmp/bridge.log  "$LOG_DIR/bridge.log"  2>/dev/null || true

info "启动 RViz..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; rosrun rviz rviz -d /catkin_ws/src/nav_demo/rviz/minimal.rviz >/tmp/rviz.log 2>&1"

sleep 2
ok "RViz 已启动 (如果没有窗口出现, 检查日志: docker exec $ROS_CONTAINER cat /tmp/rviz.log)"

# ══════════════════════════════════════
#  启动 teleop (前台)
# ══════════════════════════════════════

echo ""
echo "============================================"
echo "  Isaac Navigation Sim — 就绪!"
echo "============================================"
echo ""
ROBOT_DISPLAY="${ROBOT_TYPE:-config.yaml 中的配置}"
echo "  机器人: $ROBOT_DISPLAY"
echo "  场景: $DATASET_DIR"
echo ""
echo "  控制:"
echo "    W/S — 前进/后退"
echo "    A/D — 左/右平移"
echo "    Q/E — 左转/右转"
echo "    R   — 重置机器人"
echo "    Space — 紧急停止"
echo "    Ctrl+C — 退出并停止所有"
echo ""
echo "  确保此终端有键盘焦点!"
echo "============================================"
echo ""

# 启动 teleop (前台, Ctrl+C 触发清理)
docker exec -it "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash && rosrun nav_demo teleop_keyboard.py _linear_speed:=1.5 _lateral_speed:=1.0 _angular_speed:=1.5"
