#!/bin/bash
# check_env.sh - 环境检查脚本

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}✓${NC} $*"; }
err()  { echo -e "${RED}✗${NC} $*"; }
warn() { echo -e "${YELLOW}!${NC} $*"; }

echo "Isaac Navigation Sim - 环境检查"
echo "================================"
echo ""

ERRORS=0

# Docker
echo -n "检查 Docker... "
if command -v docker &>/dev/null && docker info &>/dev/null; then
    ok "Docker 已安装且可访问"
else
    err "Docker 不可用"
    ERRORS=$((ERRORS + 1))
fi

# Docker Compose
echo -n "检查 Docker Compose... "
if docker compose version &>/dev/null; then
    ok "Docker Compose 已安装"
else
    err "Docker Compose 不可用"
    ERRORS=$((ERRORS + 1))
fi

# nvidia-smi
echo -n "检查 NVIDIA 驱动... "
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    ok "NVIDIA 驱动正常"
else
    err "NVIDIA 驱动不可用"
    ERRORS=$((ERRORS + 1))
fi

# nvidia-container-toolkit
echo -n "检查 nvidia-container-toolkit... "
if docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi &>/dev/null 2>&1; then
    ok "nvidia-container-toolkit 正常"
else
    err "nvidia-container-toolkit 不可用"
    ERRORS=$((ERRORS + 1))
fi

# Isaac Sim 镜像
echo -n "检查 Isaac Sim 镜像... "
if docker images | grep -q "isaacsim51-r2s2r"; then
    ok "isaacsim51-r2s2r:latest 存在"
else
    err "isaacsim51-r2s2r:latest 不存在"
    ERRORS=$((ERRORS + 1))
fi

# 场景文件
echo -n "检查场景文件... "
DATASET_DIR="/home/mt/dzy/tunnel-inspection-v2/dataset"
if [ -d "$DATASET_DIR" ] && [ -f "$DATASET_DIR/tunnel1.usdz" ]; then
    ok "场景文件存在: $DATASET_DIR/tunnel1.usdz"
else
    warn "场景文件不存在（可选）"
fi

# 项目文件
echo -n "检查项目结构... "
if [ -f "start.sh" ] && [ -d "r2s2r" ] && [ -d "ros1_ws" ] && [ -d "docker" ]; then
    ok "项目结构完整"
else
    err "项目结构不完整"
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "================================"
if [ $ERRORS -eq 0 ]; then
    ok "环境检查通过！可以运行 ./start.sh"
    exit 0
else
    err "环境检查失败，发现 $ERRORS 个问题"
    exit 1
fi
