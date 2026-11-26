#!/bin/bash
# 在一次性 Isaac Sim 容器里用 python.sh 跑脚本 (带 GPU + 复用 cache 卷).
# 不影响 isaac_nav_isaac compose 栈; --rm 用完即删.
#
# 用法: ./tools/_run_isaac_python.sh <容器内脚本绝对路径> [args...]
# 例:   ./tools/_run_isaac_python.sh /workspace/tools/_dump_lidar_usd.py \
#           /workspace/lidar_configs/usd/raster_120x90.usd
set -e
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm --gpus all --network host --ipc host --user root \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e ISAACSIM_PATH=/isaac-sim \
  -v "$PROJ/lidar_configs:/workspace/lidar_configs" \
  -v "$PROJ/tools:/workspace/tools:ro" \
  -v docker_isaac-cache:/isaac-sim/.cache \
  -v docker_isaac-computecache:/isaac-sim/.nv/ComputeCache \
  -v docker_isaac-config:/isaac-sim/.nvidia-omniverse/config \
  -v docker_isaac-data:/isaac-sim/.local/share/ov/data \
  -v docker_isaac-logs:/isaac-sim/.nvidia-omniverse/logs \
  -w /workspace --entrypoint /isaac-sim/python.sh \
  r2s2r-isaac:5.1.0 "$@"
