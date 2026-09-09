#!/usr/bin/env bash
# ============================================================
# record_bag.sh — AL-iLQR Go2 测试 bag 录制脚本（在 NX 上运行）
#
# 用法:
#   bash record_bag.sh <scene_name> [duration_sec] [--with-cloud]
#
# 参数:
#   scene_name    — 场景名（必填），例如: corridor, turn, dyn_obs
#   duration_sec  — 录制秒数（默认 0 = 无限制，Ctrl-C 停止）
#   --with-cloud  — 追加录制 /odin1/cloud_slam 原始点云
#                   （体积大，约 100MB/min，仅深度排查时用）
#
# 示例:
#   bash record_bag.sh corridor              # 走廊场景，Ctrl-C 停止
#   bash record_bag.sh corridor 120          # 录 120 秒自动停止
#   bash record_bag.sh dyn_obs 60 --with-cloud
#
# 输出: ~/bags/YYYYMMDD_HHMMSS_<scene>.bag
# 拷回开发机分析:
#   sshpass -p 123 scp unitree@192.168.123.18:~/bags/<file>.bag ./
# ============================================================

set -euo pipefail

SCENE="${1:-test}"
DURATION="${2:-0}"
WITH_CLOUD="${3:-}"

BAG_DIR="${HOME}/bags"
mkdir -p "${BAG_DIR}"

TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
BAG_FILE="${BAG_DIR}/${TIMESTAMP}_${SCENE}"

# --- 录制话题（al_ilqr_go2 全链路诊断集）---
TOPICS=(
  # == 控制输出（最重要）==
  "/cmd_vel"                   # 最终速度指令
  "/al_ilqr/status"            # 求解器状态字符串（cvg/耗时/viol/obs/a/s/v/w）
  "/al_ilqr/optimized_path"    # iLQR 优化轨迹

  # == 规划器输入 ==
  "/odin1/odometry"            # 里程计（位姿+实测速度）
  "/scan"                      # 前向激光（动态避障输入）
  "/scan_360"                  # fake360 聚合激光
  "/initial_path"              # map_planner 全局路径（map 系）
  "/move_base_simple/goal"     # 目标点
  "/neupan/arrive"             # 到达事件

  # == 地图与坐标系 ==
  "/map"                       # 占据栅格（回放叠加显示）
  "/tf"
  "/tf_static"

  # == 全部节点日志（含 al_ilqr 的 WARN/ERROR）==
  "/rosout"
)

# 可选: 原始点云（体积大）
if [ "${WITH_CLOUD}" = "--with-cloud" ]; then
  TOPICS+=("/odin1/cloud_slam")
fi

echo "============================================"
echo "  AL-iLQR Go2 Bag Recorder"
echo "  Scene   : ${SCENE}"
echo "  Duration: $([ "${DURATION}" -gt 0 ] && echo "${DURATION}s" || echo "unlimited (Ctrl-C to stop)")"
echo "  Cloud   : $([ "${WITH_CLOUD}" = "--with-cloud" ] && echo "YES (large!)" || echo "no")"
echo "  Output  : ${BAG_FILE}.bag"
echo "============================================"
echo ""

# 启动录制（--lz4 压缩省空间；-b 512 增大缓冲防丢帧）
if [ "${DURATION}" -gt 0 ]; then
  rosbag record -O "${BAG_FILE}" --lz4 -b 512 --duration="${DURATION}" "${TOPICS[@]}"
else
  rosbag record -O "${BAG_FILE}" --lz4 -b 512 "${TOPICS[@]}"
fi

echo ""
echo "[完成] bag 已保存: ${BAG_FILE}.bag"
echo "[大小] $(du -sh "${BAG_FILE}".bag* | cut -f1)"
echo ""
echo "-- 快速验证 : rosbag info ${BAG_FILE}.bag"
echo "-- 拷回开发机: sshpass -p 123 scp unitree@192.168.123.18:${BAG_FILE}.bag ./"
