# Isaac Navigation Sim - 优化配置（提高流畅度）
# 
# 使用方法：
# ./start.sh --robot kinematic_car
# 然后手动替换容器内的配置，或者重启时使用此配置

## 优化建议

### 1. 降低渲染质量以提高仿真帧率
simulation:
  width: 1024          # 降低分辨率 (原 1280)
  height: 576          # 降低分辨率 (原 720)

### 2. 提高点云频率但降低点数
bridge:
  pointcloud_rate: 10  # 提高到 10 Hz (原 5 Hz)

### 3. 使用更低分辨率的 LiDAR
# 编辑 calibration/kinematic_car/intrinsics_lidar1.yaml
# 改为更简单的配置，比如 "Example_Rotary"

### 4. 在 RViz 中优化点云显示
# PointCloud2 显示设置：
# - Size (m): 0.02 -> 0.05 (更大的点，渲染更快)
# - Decay Time: 0 (不保留历史帧)
# - Style: Points -> Squares (渲染更快)

### 5. 确认不是键盘控制的问题
# 可能是按键响应慢，不是机器人卡
# 测试：持续按住 W 键 2-3 秒，看是否流畅

## 诊断步骤

1. 检查实际仿真帧率：
   docker logs isaac_nav_isaac | grep fps

2. 检查机器人位姿更新频率（应该 50 Hz）：
   docker exec -it isaac_nav_ros1 bash
   source /catkin_ws/devel/setup.bash
   rostopic hz /state_estimation

3. 如果位姿是 50 Hz，说明机器人运动是流畅的，
   只是点云可视化慢（1.7 Hz）

## 现状分析

当前配置：
- 位姿发布：~50 Hz ✅ 流畅
- 点云发布：~1.7 Hz ❌ 慢（配置是 5 Hz）
- GPU 使用率：85% ⚠️ 较高

结论：
- 机器人本身运动应该是流畅的（50 Hz 位姿更新）
- "卡顿"主要是 RViz 中点云更新慢导致的视觉效果
- 对于导航算法来说，1.7 Hz 的点云已经够用

## 权衡

- **不优化**：导航算法能用，只是看起来卡
- **优化点云**：提高到 5-10 Hz，但可能 GPU 负载更高
- **降低分辨率**：提高帧率，但画质降低

建议：先测试机器人实际运动是否流畅（持续按 W 键），
如果流畅，就不需要优化；如果确实卡顿，再调整配置。
