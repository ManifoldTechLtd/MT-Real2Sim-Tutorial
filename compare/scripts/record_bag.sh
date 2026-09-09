#!/usr/bin/env bash
# ============================================================
# record_bag.sh — 统一 bag 录制脚本
# 适用于: 方案 B (vanilla NeuPAN) 和 方案 C (manifold NeuPAN fork)
# 用法:
#   bash record_bag.sh <scene_name> [method_label] [duration_sec]
#
# 参数:
#   scene_name   — 场景名, 例如: corridor, static_obs, dynamic_obs
#   method_label — 方法标签, 例如: vanilla 或 manifold (默认: unknown)
#   duration_sec — 录制秒数 (默认: 0 = 无限制, Ctrl-C 停止)
#
# 示例:
#   bash record_bag.sh corridor manifold 60      # 走廊 + manifold 方法, 录 60s
#   bash record_bag.sh dynamic_obs vanilla        # 动态障碍 + vanilla, 无限时间
#   bash record_bag.sh static_obs                 # 仅给场景名, 自动 Ctrl-C 停
#
# 输出文件命名: bags/YYYYMMDD_HHMMSS_<scene>_<method>.bag
# ============================================================

set -euo pipefail

SCENE="${1:-test}"
METHOD="${2:-unknown}"
DURATION="${3:-0}"

# --- 输出目录 ---
BAG_DIR="/home/unitree/code/OAYN/bags"
mkdir -p "${BAG_DIR}"

TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
BAG_FILE="${BAG_DIR}/${TIMESTAMP}_${SCENE}_${METHOD}"

# --- 录制的 topic 列表 (方案 B 和 C 的并集) ---
TOPICS=(
  # == 核心控制量 ==
  "/cmd_vel"                      # 发给 robot 的最终速度 (最重要对比量)
  "/odin1/odometry"               # 机器人实测速度/位姿

  # == 规划器输入 ==
  "/scan_360"                     # fake360 360° LaserScan (方案 B/C 共用)
  "/initial_path"                 # map_planner A* 路径
  "/move_base_simple/goal"        # rviz 目标点
  "/neupan/goal"                  # 方案 C 的内部目标话题 (可能与上重复)

  # == 方案 C 特有 ==
  "/neupan/arrive"                # 到达信号 (方案 C 自动 replan 触发)

  # == NeuPAN 输出 / 可视化 (两个方案共用) ==
  "/neupan_plan"                  # NeuPAN 规划路径
  "/neupan_initial_path"          # 初始路径可视化
  "/dune_point_markers"           # DUNE 障碍点
  "/nrmp_point_markers"           # NRMP 点
  "/robot_marker"                 # 机器人 marker
  "/neupan_ref_state"             # 参考状态点

  # == 前端 / 地图 ==
  "/map"                          # occupancy grid (用于回放时叠加显示)

  # == 变换 ==
  "/tf"
  "/tf_static"

  # == 日志 (调试) ==
  "/rosout"
)

echo "============================================"
echo "  NeuPAN Bag Recorder"
echo "  Scene   : ${SCENE}"
echo "  Method  : ${METHOD}"
echo "  Duration: $([ "${DURATION}" -gt 0 ] && echo "${DURATION}s" || echo "unlimited (Ctrl-C to stop)")"
echo "  Output  : ${BAG_FILE}.bag"
echo "============================================"
echo ""
echo "等待 3 秒后开始录制..."
sleep 3

# 拼 topic 参数
TOPIC_ARGS=""
for t in "${TOPICS[@]}"; do
  TOPIC_ARGS="${TOPIC_ARGS} ${t}"
done

# 启动录制
if [ "${DURATION}" -gt 0 ]; then
  rosbag record -O "${BAG_FILE}" --duration="${DURATION}" ${TOPIC_ARGS}
else
  rosbag record -O "${BAG_FILE}" ${TOPIC_ARGS}
fi

echo ""
echo "[完成] bag 已保存: ${BAG_FILE}.bag"
echo "[大小] $(du -sh "${BAG_FILE}.bag" | cut -f1)"
echo ""
echo "-- 快速验证: rosbag info ${BAG_FILE}.bag"
