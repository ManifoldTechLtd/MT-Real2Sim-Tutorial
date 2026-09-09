#!/bin/bash
# run_isaac.sh — 在 conda isaac_arm 环境中运行 Isaac Sim Python 脚本
# 本脚本使用场景：在本地开发环境中运行 Isaac Sim 脚本，不使用 Docker
# 
# 用法:
#   ./tools/run_isaac.sh <script.py> [args...]
#
# 示例:
#   ./tools/run_isaac.sh tests/isaac_minimal.py
#   ./tools/run_isaac.sh r2s2r/run_sim.py --config r2s2r/configs/spot.yaml
#
# 原理:
#   conda activate isaac_arm  — 复用 Python 3.11 + 已安装的依赖 (msgpack, numpy, yaml, ...)
#   source ~/isaacsim/setup_conda_env.sh — 把 Isaac Sim 扩展加入 PYTHONPATH

set -e

# 定位 conda
if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
else
    echo "[run_isaac] ERROR: 未找到 conda"
    exit 1
fi

# 激活环境
conda activate isaac_arm

# 加载 Isaac Sim 扩展路径
source "$HOME/isaacsim/setup_conda_env.sh"

# 项目根目录加入 PYTHONPATH (便于 import r2s2r.*)
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# 运行
exec python "$@"
