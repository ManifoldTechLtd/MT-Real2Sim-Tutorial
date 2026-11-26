#!/bin/bash

# 本脚本用于在 Docker 容器中运行 Isaac Sim 脚本。前提：安装的是issac sim docker版

set -e

CONTAINER_NAME="isaacsim51"

docker exec -it \
-w /projects/isaac_real2sim2real_go2 \
$CONTAINER_NAME \
bash -lc '
export PYTHONPATH=/projects/isaac_real2sim2real_go2:/projects/k02_nav:$PYTHONPATH
cd /projects/isaac_real2sim2real_go2
/isaac-sim/python.sh "$@"
' bash "$@"