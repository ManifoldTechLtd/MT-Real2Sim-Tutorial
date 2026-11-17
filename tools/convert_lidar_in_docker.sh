#!/bin/bash
# 在一次性 Isaac Sim 容器里把 lidar JSON 转成 USD.
# 无需预先 docker-compose up; 用 --rm 临时容器, 不影响 isaac_nav_isaac 栈.
#
# 用法:
#   ./tools/convert_lidar_in_docker.sh <json名或路径>
# 例:
#   ./tools/convert_lidar_in_docker.sh raster_120x90.json
#
# 说明:
#   - lidar_configs 以 rw 挂载, USD 直接落宿主机 lidar_configs/usd/<stem>.usd
#   - 复用 compose 的 named cache 卷 (docker_isaac-*) 加速 Isaac Sim 启动
#   - ISAACSIM_PATH=/isaac-sim 修正 convert_lidar_json.py 默认找 ~/isaacsim 的问题
set -e

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JSON_ARG="${1:?用法: $0 <json名, 如 raster_120x90.json>}"

# 只给文件名/相对名 → 补全为容器内路径
case "$JSON_ARG" in
  /*)  CJ="$JSON_ARG" ;;
  */*) CJ="/workspace/lidar_configs/json/$(basename "$JSON_ARG")" ;;
  *)   CJ="/workspace/lidar_configs/json/$JSON_ARG" ;;
esac

echo "[convert] PROJ=$PROJ"
echo "[convert] 容器内 JSON: $CJ"
echo "[convert] 启动一次性 Isaac Sim 容器 (首次启动可能需 1-3 分钟)..."

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
  r2s2r-isaac:5.1.0 \
  /workspace/tools/convert_lidar_json.py "$CJ"

echo "[convert] 完成. 宿主机 USD:"
ls -la "$PROJ/lidar_configs/usd/"
