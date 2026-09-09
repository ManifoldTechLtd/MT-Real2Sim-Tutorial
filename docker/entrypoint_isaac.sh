#!/bin/bash
# Isaac Sim 容器启动脚本
# 在容器内执行仿真

set -e

echo "[container] Starting Isaac Sim..."

# 设置 PYTHONPATH
export PYTHONPATH=/workspace:${PYTHONPATH}

# 进入工作目录
cd /workspace

# 执行 Python 脚本
echo "[container] Executing: /isaac-sim/python.sh r2s2r/run_sim.py --config r2s2r/configs/config.yaml"
exec /isaac-sim/python.sh r2s2r/run_sim.py --config r2s2r/configs/config.yaml
