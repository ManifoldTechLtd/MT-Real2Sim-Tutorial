#!/bin/bash
# start.sh — One-click launch for Isaac Real2Sim2Real Go2
#
# Usage:
#   ./start.sh                          # Start all (Isaac Sim + ROS + RViz + Teleop)
#   ./start.sh --stop                   # Stop all containers
#   ./start.sh --scene ~/my_scenes      # Specify scene file directory
#
# Flow:
#   1. Pre-flight checks (Docker, GPU, NGC)
#   2. Start dual containers (Isaac Sim + ROS1)
#   3. Wait for Isaac Sim ready (TCP:9090)
#   4. Start RViz (background)
#   5. Start teleop (foreground, WASD control)
#   6. Auto-cleanup on Ctrl+C

set -e

# ── Project root ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
COMPOSE_FILE="$PROJECT_DIR/docker-full/docker-compose.yml"
ROS_CONTAINER="r2s2r_ros1"
ISAAC_CONTAINER="r2s2r_isaac"
SCENE_DIR="${SCENE_DIR:-$HOME/dataset/isaac}"
TCP_PORT=9090
# First launch pulls ~20GB image + compiles shaders; override with MAX_WAIT env var
MAX_WAIT="${MAX_WAIT:-600}"
LOG_DIR="${LOG_DIR:-/tmp/r2s2r}"
mkdir -p "$LOG_DIR"

# ── Colored output ──
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

info()  { echo -e "${CYAN}[start]${NC} $*"; }
ok()    { echo -e "${GREEN}[start]${NC} $*"; }
warn()  { echo -e "${YELLOW}[start]${NC} $*"; }
err()   { echo -e "${RED}[start] ERROR:${NC} $*" >&2; }

# ── Argument parsing ──
ACTION="start"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --stop)
            ACTION="stop"
            shift
            ;;
        --scene)
            SCENE_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: ./start.sh [options]"
            echo ""
            echo "Options:"
            echo "  --stop            Stop all containers"
            echo "  --scene <dir>     Specify scene file directory (default: ~/dataset/isaac)"
            echo "  -h, --help        Show this help"
            exit 0
            ;;
        *)
            err "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# ── Stop ──
if [ "$ACTION" = "stop" ]; then
    info "Stopping containers..."
    if ! docker info &>/dev/null; then
        err "Cannot access Docker (not running or current user lacks permission)"
        err "  Verify: systemctl is-active docker; groups | grep -w docker"
        exit 1
    fi
    docker compose -f "$COMPOSE_FILE" down
    ok "Stopped"
    exit 0
fi

# ══════════════════════════════════════
#  Pre-flight checks
# ══════════════════════════════════════

info "Running pre-flight checks..."

# Docker
if ! command -v docker &>/dev/null; then
    err "Docker not found. Install: sudo apt-get install docker.io docker-compose-v2"
    exit 1
fi

if ! docker info &>/dev/null; then
    # Distinguish: service not running vs permission denied
    if ! systemctl is-active --quiet docker 2>/dev/null; then
        err "Docker service not running. Run: sudo systemctl start docker"
    elif ! groups | grep -qw docker; then
        err "Current user is not in the docker group. Run:"
        err "  sudo usermod -aG docker \$USER"
        err "  Then log out and back in, or run: newgrp docker"
    else
        err "Cannot access Docker. Run 'docker info' for details"
    fi
    exit 1
fi

# GPU
if ! command -v nvidia-smi &>/dev/null; then
    err "nvidia-smi not found. Install NVIDIA driver (>= 560)"
    exit 1
fi

if ! nvidia-smi &>/dev/null; then
    err "GPU driver error. Run nvidia-smi to check"
    exit 1
fi

# nvidia-container-toolkit
if ! docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi &>/dev/null 2>&1; then
    warn "Docker GPU passthrough test failed. Ensure nvidia-container-toolkit is installed:"
    warn "  sudo apt-get install nvidia-container-toolkit"
    warn "  sudo nvidia-ctk runtime configure --runtime=docker"
    warn "  sudo systemctl restart docker"
    # Don't exit; let user decide whether to continue
fi

# NGC login check (try real access first, fall back to config.json grep)
NGC_OK=0
if docker manifest inspect nvcr.io/nvidia/isaac-sim:5.1.0 &>/dev/null; then
    NGC_OK=1
elif grep -q "nvcr.io" ~/.docker/config.json 2>/dev/null; then
    # config.json contains nvcr.io (may be credential helper) — treat as logged in
    NGC_OK=1
fi
if [ "$NGC_OK" -eq 0 ]; then
    warn "NGC login not detected (or local isaac-sim:5.1.0 image not found). First run requires login:"
    warn "  1. Register NGC account: https://ngc.nvidia.com/"
    warn "  2. Generate API Key: NGC website → Setup → API Key"
    warn "  3. Run: docker login nvcr.io --username '\$oauthtoken' --password <YOUR_KEY>"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Scene files
if [ ! -d "$SCENE_DIR" ]; then
    warn "Scene directory does not exist: $SCENE_DIR"
    warn "Simulation will use default blank ground. Obtain scene files from team documentation."
    echo ""
fi

ok "Pre-flight checks passed"

# ══════════════════════════════════════
#  Cleanup function
# ══════════════════════════════════════

cleanup() {
    echo ""
    info "Shutting down..."

    # Kill ROS processes inside container (RViz/roscore/bridge all run there)
    docker exec "$ROS_CONTAINER" bash -c \
        "killall -q rviz roscore rosmaster rosout roslaunch 2>/dev/null; true" 2>/dev/null || true

    # Stop containers
    docker compose -f "$COMPOSE_FILE" down 2>/dev/null || true

    ok "Stopped. Goodbye!"
}
trap cleanup EXIT

# ══════════════════════════════════════
#  Start containers
# ══════════════════════════════════════

info "Starting containers (Isaac Sim + ROS1)..."
xhost +local:docker >/dev/null 2>&1 || true
SCENE_DIR="$SCENE_DIR" docker compose -f "$COMPOSE_FILE" up -d

# ══════════════════════════════════════
#  Wait for Isaac Sim ready
# ══════════════════════════════════════

info "Waiting for Isaac Sim to start (listening TCP:$TCP_PORT, timeout ${MAX_WAIT}s)..."
# Detect cold start (check if Vulkan shader cache directory is non-empty)
# Uses the already-running isaac container, no external image needed
CACHE_NONEMPTY=$(docker exec "$ISAAC_CONTAINER" \
    bash -c 'ls /isaac-sim/.nv/ComputeCache 2>/dev/null | head -1' 2>/dev/null)
if [ -z "$CACHE_NONEMPTY" ]; then
    warn "First cold start: compiling Vulkan shaders, typically 5-10 min. Be patient, don't Ctrl+C"
    warn "If default ${MAX_WAIT}s timeout is not enough: MAX_WAIT=1800 ./start.sh"
else
    info "Shader cache found, expect 30-60 seconds to ready"
fi

WAITED=0
while ! ss -tlnp 2>/dev/null | grep -q ":${TCP_PORT} " && [ "$WAITED" -lt "$MAX_WAIT" ]; do
    sleep 3
    WAITED=$((WAITED + 3))
    if [ $((WAITED % 15)) -eq 0 ]; then
        info "Waited ${WAITED}s / ${MAX_WAIT}s ..."
    fi
done

if [ "$WAITED" -ge "$MAX_WAIT" ]; then
    err "Timed out waiting for Isaac Sim (${MAX_WAIT}s). Check:"
    err "  docker logs $ISAAC_CONTAINER"
    err "If first launch is still pulling images, increase timeout: MAX_WAIT=1800 ./start.sh"
    exit 1
fi

ok "Isaac Sim TCP port is open ($TCP_PORT)"

# ── Wait for Isaac Sim simulation loop to be running ──
info "Waiting for Isaac Sim to initialize (checking container logs)..."
info ">>> Click the PLAY button in the Isaac Sim window when it appears <<<"
DATA_WAIT=0
DATA_MAX_WAIT=300
while [ "$DATA_WAIT" -lt "$DATA_MAX_WAIT" ]; do
    # Check Isaac container logs for diagnostic output (printed every 5s in main loop)
    if docker logs "$ISAAC_CONTAINER" 2>&1 | grep -q "\[diag\]"; then
        break
    fi
    sleep 3
    DATA_WAIT=$((DATA_WAIT + 3))
    if [ $((DATA_WAIT % 15)) -eq 0 ]; then
        # Show latest status from Isaac logs
        LAST_LINE=$(docker logs "$ISAAC_CONTAINER" 2>&1 | grep "\[run_sim\]\|\[diag\]" | tail -1)
        info "Waiting ${DATA_WAIT}s... last Isaac log: ${LAST_LINE:-<initializing>}"
    fi
done

if [ "$DATA_WAIT" -ge "$DATA_MAX_WAIT" ]; then
    warn "Timed out waiting for Isaac Sim to enter main loop (${DATA_MAX_WAIT}s)"
    warn "Continuing anyway — bridge will auto-reconnect when ready"
fi

ok "Isaac Sim simulation loop is running"

# ══════════════════════════════════════
#  Start ROS: bridge + RViz + teleop
# ══════════════════════════════════════

# Start roscore + bridge (background inside container, logs written to disk for debugging)
# Note: in bash `cmd1 && cmd2 & cmd3` puts cmd1 && cmd2 in a subshell, breaking source;
# therefore use ';' after source, and run roscore separately in background
info "Starting roscore..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roscore >/tmp/roscore.log 2>&1"

# Wait for roscore to be ready (check port 11311 inside container via python)
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
    err "roscore failed to start within 30s. Check: docker exec $ROS_CONTAINER cat /tmp/roscore.log"
    exit 1
fi
ok "roscore ready (${ROSCORE_WAIT}s)"

info "Starting sim_bridge (log: $LOG_DIR/bridge.log)..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; roslaunch sim_bridge bridge.launch --wait >/tmp/bridge.log 2>&1"
sleep 3
docker cp "$ROS_CONTAINER":/tmp/roscore.log "$LOG_DIR/roscore.log" 2>/dev/null || true
docker cp "$ROS_CONTAINER":/tmp/bridge.log  "$LOG_DIR/bridge.log"  2>/dev/null || true

# Start RViz (background in container; note: docker exec -d cannot retrieve RViz PID on host,
# cleanup is handled uniformly by docker compose down on exit)
info "Starting RViz..."
docker exec -d "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash; rosrun rviz rviz -d /catkin_ws/src/nav_demo/rviz/minimal.rviz >/tmp/rviz.log 2>&1"

sleep 2
ok "RViz started (if no window appears, check logs in $LOG_DIR or: docker exec $ROS_CONTAINER cat /tmp/rviz.log)"

# ══════════════════════════════════════
#  Start teleop (foreground)
# ══════════════════════════════════════

echo ""
echo "============================================"
echo "  Isaac Real2Sim2Real Go2 — Ready!"
echo "============================================"
echo ""
echo "  Controls:"
echo "    W/S — Forward/Backward"
echo "    A/D — Strafe Left/Right"
echo "    Q/E — Turn Left/Right"
echo "    R   — Reset robot"
echo "    Space — Emergency stop"
echo "    Ctrl+C — Quit and stop all"
echo ""
echo "  Make sure this terminal has keyboard focus!"
echo "============================================"
echo ""

# Start teleop in foreground (Ctrl+C triggers cleanup)
docker exec -it "$ROS_CONTAINER" bash -c \
    "source /catkin_ws/devel/setup.bash && rosrun nav_demo teleop_keyboard.py _linear_speed:=3.0 _lateral_speed:=2.0 _angular_speed:=3.0"
