#!/bin/bash
# run_isaac_docker.sh — Run a Python script inside the Docker Isaac Sim container
#
# Usage:
#   ./tools/run_isaac_docker.sh <script.py> [args...]
#
# Examples:
#   ./tools/run_isaac_docker.sh r2s2r/run_sim.py --config r2s2r/configs/config.yaml
#   ./tools/run_isaac_docker.sh r2s2r/run_sim.py --headless
#   ./tools/run_isaac_docker.sh tests/isaac_minimal.py

set -e

CONTAINER_NAME="r2s2r_isaac"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[run_isaac_docker] ERROR: Container ${CONTAINER_NAME} is not running"
    echo "[run_isaac_docker] Start first: docker compose -f docker-full/docker-compose.yml up -d"
    exit 1
fi

# Convert host relative path to container /workspace/ path
SCRIPT="$1"
shift
if [[ "$SCRIPT" != /* ]]; then
    SCRIPT="/workspace/$SCRIPT"
fi

exec docker exec -it "$CONTAINER_NAME" python "$SCRIPT" "$@"
