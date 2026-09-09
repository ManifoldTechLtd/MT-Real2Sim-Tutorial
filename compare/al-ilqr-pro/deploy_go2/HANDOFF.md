# AL-iLQR Go2 项目交接文档

## 项目概述
- AL-iLQR-Pro 轨迹优化器适配宇树 Go2 四足机器人
- 自包含 catkin 包，部署在 Go2 NX 电脑（unitree@192.168.123.18，密码123）
- NX 工作区：~/code_ilqr/（overlay 在 ~/k02_nav/ros_ws_base_inspection/ros_ws/ 之上）
- 只需 source ~/code_ilqr/devel/setup.bash 即可同时访问所有包

## 系统架构
```
odin_ros_driver（k02_nav，不可移植）→ /odin1/odometry, /odin1/cloud_slam, TF(map→odom→odin1_base_link)
pointcloud_to_laserscan（code_ilqr/map_planner/whole.launch）→ /scan (90° FOV, frame=odin1_base_link)
map_server → /map (map_opt.pgm)
map_planner → /initial_path (A*+平滑，frame=map)
fake360 → /scan_360, /fake_map
al_ilqr_go2 → 订阅 /initial_path + /odin1/odometry + /scan → TF变换path到odom系 → iLQR求解 → /cmd_vel
vel_to_sdk（unitree_control，code_ilqr）→ Go2 SDK
```

## 当前状态（2026-07-30 真机测试）

### 已验证通过
- Phase 1 环境检查：全通过
- Phase 2 编译部署：通过（Release -O3，catkin config --extend overlay）
- Phase 3 静态验证：
  - odin driver 正常（odom 10Hz, TF 完整, cloud_slam 正常）
  - /scan, /scan_360 正常（10Hz）
  - map_planner 规划成功（需机器人在地图自由区）
  - al_ilqr 求解收敛：**2.1-2.5ms**（解析导数生效），obs=5-9, viol=0.000
- Phase 4 手动 cmd_vel 测试：发 linear.x=0.2 持续 2s，**Go2 确实向前冲了一段**——底盘链路通畅

### 当前阻塞问题（✅ 已修复并于 07-30 续3 真机验证通过，端到端导航闭环成功）

> 真机结果：发目标点后 a=1.005、v=0.53-0.62 m/s，机器人自主行驶 5.2m 到点，
> `goal reached (dist=0.293m)` 自动停车并发布 /neupan/arrive。详见 PROGRESS.md 续3。
>
> ⚠️ 安全发现：odin1_ros1.launch 自带的 host_sdk_sample 直接订阅 /cmd_vel 转发
> Go2 SDK，底盘常通，use_unitree_control:=false 挡不住。静态验证前必须
> `rostopic info /cmd_vel` 确认无订阅者，或重映射到 /cmd_vel_test。

**原问题：al_ilqr 求解器输出的前进速度接近零（v≈0.001-0.007 m/s），机器人不走**

现象：
- 求解器收敛（cvg=1），但加速度输出 a≈-0.001（几乎为零）
- bicycleToDiff 映射后 cmd_vel linear.x ≈ 0.001（死区内）
- 求解器给了 steering=0.142（想转向）但不想前进
- 之前还有间歇性不收敛（已通过增大迭代和减少障碍数解决）

根因（已于 07-30 续2 代码走查确认，原“ref_speed 未传入代价函数”的猜测不成立，
该链路实际完好），三问题叠加：
1. **墙壁外接圆堵死走廊（主因）**：ObstacleAdapter 把整段墙聚成一簇取外接圆，
   圆心在墙上、半径可达 2m，圆体覆盖走廊中央→任何前进都违约，
   “原地不动”成为零违约最优解（与 cvg=1、viol=0、只转不走完全吻合）
2. **warm start 自锁**：近零解进入 warm start 后 Pure-Pursuit 前向初值永不再生效
3. **速度自举死锁**：v_cmd = odom速度 + a·dt，静止时最多 0.1 m/s（死区内），
   机器人不动则 odom 恒 0，永远爬不出死区

已修复（本机代码，编译验证通过，**尚未同步 NX**）：
- obstacle_adapter.hpp：大簇拆分为贴墙小圆串（新参数 dynamic_obstacles/max_cluster_radius=0.4）
- bicycle_to_diff.hpp + 节点：cmd_vel 改取规划轨迹前视速度 states[k](3)
 （新参数 control_mapping/speed_lookahead_steps=3，起步指令直接 ≈0.3 m/s）
- 节点停滞恢复：连续 3 周期近零速→清 warm start 回退 Pure-Pursuit 初值
 （新参数 planning/stall_cmd_threshold=0.05、planning/stall_recovery_cycles=3）

### 已同步进交付默认值的 NX 实测参数（无需再在 NX 手改）
```yaml
# config/al_ilqr_go2.yaml 默认值已更新：
solver:
  max_outer_iterations: 6    # 原4，07-30 真机验证
  max_inner_iterations: 30   # 原12，07-30 真机验证
dynamic_obstacles:
  max_obstacles: 5           # 原10，07-30 真机验证
```

## 下一步计划（Go2 开机后）

1. rsync 本机 deploy_go2/ 到 NX（见下方命令），catkin build Release
2. 启动 odin driver（需重定位：把狗遥控到地图空旷区）
3. 启动 whole.launch（use_unitree_control:=false）+ al_ilqr_go2
4. 静态验证：发目标点，确认：
   - /al_ilqr/status 中 obs 数量（拆分后变多但单个变小，取最近 5 个）
   - cmd_vel linear.x 起步 ≥ 0.2 m/s（明确脱离死区），不再出现连续近零速
   - 若仍停滞，日志中应能看到 "clearing warm start to escape standstill" 告警
5. 确认后启动 unitree_control 做真正的 Phase 4 低速直线测试

### 重定位注意事项
- Odin driver 使用 k02_nav 的 launch：`source ~/k02_nav/ros_ws_base_inspection/ros_ws/devel/setup.bash && roslaunch odin_ros_driver odin1_ros1.launch`
- 加载重定位地图 map_merged.bin 后需用遥控器移动 Go2 到地图空旷区完成重定位（map→odom TF 稳定）
- 上次的占据区问题：map 坐标 (-20, -1.2) 附近可能是墙壁膨胀区，需要确保定位在走廊正中间

## 关键文件清单

| 文件 | 位置（本机） | 作用 |
|------|------------|------|
| ROS 节点主体 | deploy_go2/src/al_ilqr_ros_node.cpp | 整个规划循环、参数加载、话题订阅发布 |
| 路径转换器 | deploy_go2/include/al_ilqr_ros/path_converter.hpp | 全局路径 → guidance 采样 |
| iLQR 求解器 | deploy_go2/src/autodrive/guidance_tracker.cpp | Track() 函数、代价类定义、约束注入 |
| 障碍物适配 | deploy_go2/include/al_ilqr_ros/obstacle_adapter.hpp | /scan → 圆形障碍物聚类 |
| 配置文件 | deploy_go2/config/al_ilqr_go2.yaml | 所有参数 |
| Launch | deploy_go2/launch/al_ilqr_go2.launch | 节点启动、参数加载 |
| 部署手册 | deploy_go2/DEPLOY.md | Phase 0-7 测试流程 |
| 进度记录 | deploy_go2/PROGRESS.md | 开发复盘日志 |
| NX whole.launch | (NX) ~/code_ilqr/src/map_planner/launch/whole.launch | 导航栈启动（含 pc_to_scan, fake360, map_server, map_planner） |

## SSH 快速命令参考

```bash
# 连接
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.18

# NX 上环境
source /opt/ros/noetic/setup.bash && source ~/code_ilqr/devel/setup.bash

# 编译 al_ilqr_go2
cd ~/code_ilqr && catkin build al_ilqr_go2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 启动 odin driver（k02）
source ~/k02_nav/ros_ws_base_inspection/ros_ws/devel/setup.bash && roslaunch odin_ros_driver odin1_ros1.launch

# 启动导航栈（不含底盘控制）
source ~/code_ilqr/devel/setup.bash && roslaunch map_planner whole.launch use_unitree_control:=false

# 启动 al_ilqr
source ~/code_ilqr/devel/setup.bash && roslaunch al_ilqr_go2 al_ilqr_go2.launch

# 发目标点
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: -20.0, y: 3.8, z: 0.0}, orientation: {z: 0.690, w: 0.723}}}' --once

# 手动测试底盘
timeout 2 rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}' -r 10

# 停止所有
rosnode kill -a; pkill -f 'roslaunch\|roscore\|host_sdk_sample'

# 同步代码（从开发机）
sshpass -p 123 rsync -az --delete /home/yu/article_and_warehouse/algo-warehouse/compare/al-ilqr-pro/deploy_go2/ unitree@192.168.123.18:/home/unitree/code_ilqr/src/al_ilqr_go2/
```

## 安全须知
- Phase 4+ 启动 unitree_control（vel_to_sdk）后机器人会真正运动
- 急停：遥控器 L1+L2 硬急停 > 杀 vel_to_sdk 节点 > 发零速度覆盖
- 测试时 max_linear_vel 先从 0.3 开始
- whole.launch 中的 use_unitree_control:=false 参数控制是否接入底盘
