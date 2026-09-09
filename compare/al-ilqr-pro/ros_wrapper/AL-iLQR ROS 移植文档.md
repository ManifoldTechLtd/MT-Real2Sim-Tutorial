# AL-iLQR ROS 移植文档

## 1. 项目概述

本项目是 **AL-iLQR-Pro** 求解器的 ROS Noetic 封装包，用于在 Jetson NX 上接收 Odin 导航栈的里程计、全局路径与激光雷达数据，周期性求解最优控制，并将自行车模型控制量映射为差速驱动 `cmd_vel` 输出给 Go2 运动控制。

## 2. 架构说明

```
Odin传感器 → [odometry/TF/scan] → AL-iLQR ROS Node → [cmd_vel] → Go2运控
                                             ↑
                             map_planner [global_path]
```

数据流说明：

1. `al_ilqr_ros_node` 订阅 `/odin1/odometry`、TF(`odom` → `odin1_base_link`)、`/map_planner/global_path` 与 `/scan`。
2. `PathConverter` 从全局路径中提取 `GuidancePoint` 序列作为局部参考轨迹。
3. `GuidanceTracker::Track()` 在预测时域 `horizon` 内求解最优状态-控制序列。
4. 取第一步控制量 `[accel, steering]`，经 `bicycleToDiff()` 映射为 `[v_linear, omega]`。
5. 发布 `geometry_msgs/Twist` 到 `/cmd_vel`，同时发布 `/al_ilqr/optimized_path` 用于可视化。

## 3. 编译步骤（在 NX 上）

### 3.1 前提条件

- Ubuntu 20.04 / ROS Noetic（已配置 `ros-noetic-desktop` 或 `ros-noetic-ros-base`）
- `catkin_tools`：
  ```bash
  sudo apt update
  sudo apt install python3-catkin-tools
  ```
- 系统依赖：
  ```bash
  sudo apt install libeigen3-dev
  ```
- 已存在 catkin workspace，并将本包放入 `src/`：
  ```bash
  cd ~/catkin_ws/src
  ln -s /path/to/algo-warehouse/compare/al-ilqr-pro/ros_wrapper al_ilqr_ros
  ```

### 3.2 catkin build 命令

```bash
cd ~/catkin_ws
catkin build al_ilqr_ros
source devel/setup.zsh   # 或 setup.bash
```

> 提示：本包通过相对路径 `../` 引用 AL-iLQR-Pro 上游核心代码，因此 **必须保持 `ros_wrapper/` 目录位于 `al-ilqr-pro/ros_wrapper/`，不得单独移动**。

## 4. 启动命令

### 4.1 单独启动

```bash
roslaunch al_ilqr_ros al_ilqr_planner.launch
```

启动文件已预置默认 topic 与 frame：

- `odom_topic`: `/odin1/odometry`
- `path_topic`: `/map_planner/global_path`
- `scan_topic`: `/scan`
- `cmd_vel_topic`: `/cmd_vel`
- `base_frame`: `odin1_base_link`
- `odom_frame`: `odom`

### 4.2 配合 Odin-Nav-Stack 启动

在启动 Odin 的 `nav_algo` / `map_planner` 节点后，再启动 AL-iLQR 节点：

```bash
# 终端 1：启动 Odin-Nav-Stack（示例）
roslaunch nav_algo odin_nav.launch

# 终端 2：启动 AL-iLQR 规划器
roslaunch al_ilqr_ros al_ilqr_planner.launch
```

确保 `/map_planner/global_path` 已发布，且 TF 树中存在 `odom` → `odin1_base_link` 变换。

## 5. 参数说明

关键参数在 `config/al_ilqr_params.yaml` 中配置：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `vehicle/wheelbase` | 0.4 | 车辆轴距（m） |
| `vehicle/max_speed` | 1.0 | 最大纵向速度（m/s） |
| `vehicle/min_speed` | -0.3 | 最小纵向速度（m/s） |
| `vehicle/max_acceleration` | 1.5 | 最大加速度（m/s²） |
| `vehicle/min_acceleration` | -2.0 | 最大减速度（m/s²） |
| `vehicle/max_steering` | 0.6 | 最大前轮转角（rad） |
| `vehicle/min_steering` | -0.6 | 最小前轮转角（rad） |
| `vehicle/length` | 0.6 | 车身长度（m） |
| `vehicle/width` | 0.4 | 车身宽度（m） |
| `vehicle/rear_axle_to_rear` | 0.15 | 后轴到车尾距离（m） |
| `solver/horizon` | 20 | 预测时域步数 |
| `solver/dt` | 0.1 | 单步时长（s） |
| `solver/max_outer_iterations` | 4 | AL 外层迭代次数 |
| `solver/max_inner_iterations` | 15 | iLQR 内层迭代次数 |
| `solver/initial_penalty` | 20.0 | 初始惩罚因子 |
| `solver/penalty_scaling` | 5.0 | 惩罚因子缩放比 |
| `solver/constraint_tolerance` | 0.1 | 约束容忍度 |
| `weights/position_weight` | 5.0 | 位置跟踪权重 |
| `weights/heading_weight` | 2.0 | 航向跟踪权重 |
| `weights/speed_weight` | 1.0 | 速度跟踪权重 |
| `weights/terminal_*_weight` | — | 终端状态各项权重 |
| `weights/accel_reg_weight` | 0.2 | 加速度正则权重 |
| `weights/steering_reg_weight` | 0.4 | 前轮转角正则权重 |
| `planning/rate` | 10.0 | 规划循环频率（Hz） |
| `planning/ref_speed` | 1.0 | 参考纵向速度（m/s） |
| `planning/guidance_lookahead` | 25 | 从全局路径向前看的点数 |
| `road/lower_bound` | -5.0 | 道路下边界（x 轴参考线，m） |
| `road/upper_bound` | 5.0 | 道路上边界（x 轴参考线，m） |
| `obstacle/safety_radius` | 0.35 | 障碍物安全半径（m） |
| `obstacle/scan_max_range` | 5.0 | 激光雷达最大有效距离（m） |
| `obstacle/clustering_distance` | 0.3 | 点云聚类距离（m） |
| `obstacle/min_obstacle_radius` | 0.1 | 最小障碍物半径（m） |
| `path/curvature_weight` | 0.5 | 曲率对速度缩放的影响权重 |
| `path/min_speed` | -0.3 | 路径规划最小速度（m/s） |
| `path/max_speed` | 1.0 | 路径规划最大速度（m/s） |
| `control_mapping/virtual_wheelbase` | 0.4 | 差速模型虚拟轴距（m） |
| `control_mapping/max_linear_vel` | 1.0 | 输出最大线速度（m/s） |
| `control_mapping/max_angular_vel` | 1.5 | 输出最大角速度（rad/s） |

## 6. 话题接口

### 6.1 订阅话题

| Topic | 类型 | 说明 |
|-------|------|------|
| `/odin1/odometry` | `nav_msgs/Odometry` | 车辆里程计，用于提取当前速度与位置 |
| `/map_planner/global_path` | `nav_msgs/Path` | 全局路径，作为局部跟踪参考 |
| `/scan` | `sensor_msgs/LaserScan` | 激光雷达数据，用于障碍物检测与状态监控 |
| TF (`odom` → `odin1_base_link`) | `tf2` | 机器人实时位姿，里程计异常时作为回退 |

### 6.2 发布话题

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 差速控制输出，`linear.x` 为线速度，`angular.z` 为角速度 |
| `/al_ilqr/optimized_path` | `nav_msgs/Path` | 优化后的预测轨迹，用于 RViz 可视化 |
| `/al_ilqr/status` | `std_msgs/String` | 节点运行状态与求解统计信息 |

## 7. 控制量映射原理

AL-iLQR 求解器内部使用 **Kinematic Bicycle Model**，控制量为：

- `accel`：纵向加速度（m/s²）
- `steering_angle`：前轮转角（rad）

Go2 运动控制接收差速命令 `[v_linear, omega]`。映射在 `include/al_ilqr_ros/bicycle_to_diff.hpp` 中实现：

```
v_linear  = current_speed + accel * dt
omega     = v_linear * tan(steering_angle) / virtual_wheelbase
```

随后对输出进行饱和限幅：

- `v_linear` 裁剪到 `[-max_linear_vel, max_linear_vel]`
- `omega` 裁剪到 `[-max_angular_vel, max_angular_vel]`

`virtual_wheelbase` 是用于差速近似的虚拟轴距，通常可与 `vehicle/wheelbase` 保持一致，但也可针对 Go2 原地转向特性单独调参。

## 8. 已知限制与后续优化方向

### 8.1 障碍物未接入 GuidanceTracker

当前 `GuidanceTracker::Track()` 接口不接收障碍物列表，因此激光雷达数据仅用于状态监控与后续扩展。实际避障目前通过以下方式间接实现：

- 检测障碍物后仅作日志记录；
- 预留安全半径 `safety_radius`，待 tracker 接口扩展后直接传入。

### 8.2 自行车模型近似的局限性

- Kinematic Bicycle Model 假设无侧滑、低速行驶，对四足平台 Go2 的高速动态或急转向场景存在模型失配。
- 前轮转角与差速角速度之间的映射为纯几何关系，未考虑 Go2 的步态控制延迟与执行器约束。

### 8.3 NX 上的性能预期

- 默认 `horizon=20`、`dt=0.1`、`rate=10Hz`，在 Jetson NX 上单帧求解时间受 iLQR 迭代次数与约束数量影响。
- 若求解耗时超过 `1/rate`，规划循环将产生抖动。建议通过降低 `horizon`、减少 `max_inner_iterations` 或关闭障碍物可视化进行性能调优。

## 9. 常见问题排查（FAQ）

### 9.1 编译失败

- **问题**：`catkin build al_ilqr_ros` 报找不到 Eigen3。
  - **解决**：`sudo apt install libeigen3-dev`，并确认 `CMakeLists.txt` 中 `find_package(Eigen3 3.3 REQUIRED NO_MODULE)`。

- **问题**：报 `al_ilqr_core` 源文件缺失。
  - **解决**：确认 `ros_wrapper/` 未单独移动；`CMakeLists.txt` 使用 `${CMAKE_CURRENT_SOURCE_DIR}/..` 引用上游源码。

- **问题**：`pcl_ros` 依赖缺失。
  - **解决**：`sudo apt install ros-noetic-pcl-ros ros-noetic-eigen-conversions`。

### 9.2 节点启动但不输出 cmd_vel

- 检查 `/al_ilqr/status`：
  - `WAITING_FOR_ODOM`：未收到里程计，确认 `odom_topic` 配置正确。
  - `WAITING_FOR_PATH`：未收到全局路径，确认 `map_planner` 已发布 `/map_planner/global_path`。
  - `GUIDANCE_TOO_SHORT`：路径转换出的 guidance 点少于 2 个，检查路径坐标系与机器人当前位姿。
  - `SOLVER_FAILED`：求解未收敛，检查车辆约束或权重设置是否合理。

- 确认 TF 树存在 `odom` → `odin1_base_link`：
  ```bash
  rosrun tf tf_echo odom odin1_base_link
  ```

### 9.3 求解超时

- 查看 `/al_ilqr/status` 中的 `solve_time_ms`。
- 降低计算负载：
  - 减小 `solver/horizon`
  - 减小 `solver/max_inner_iterations`
  - 减小 `planning/guidance_lookahead`
- 在 NX 上建议关闭不必要的可视化节点，仅保留 `/al_ilqr/optimized_path`。

## 10. AI 移植提示词记录

本次 AL-iLQR ROS 移植的关键决策如下：

1. **保持 Kinematic Bicycle Model，ROS 壳做映射**
   - 不对上游 AL-iLQR-Pro 求解器内部动力学做修改；在 ROS 节点外层将 `[accel, steering]` 映射为差速 `[v, omega]`。

2. **使用 GuidanceTracker::Track() 而非裸 ALILQRSolver**
   - 直接复用上游已封装好的 `GuidanceTracker` 高层 API，减少在 ROS 节点中手动组装 cost / constraint 的复杂度。

3. **障碍物暂以检测+预留方式实现**
   - 当前 `GuidanceTracker` 接口不接收障碍物，ROS 节点中通过 `ObstacleAdapter` 解析激光雷达并预留安全半径，待接口扩展后无缝接入。

4. **参数全部外部化**
   - 所有 topic、frame、车辆参数、求解器参数、权重、控制映射参数均通过 `config/al_ilqr_params.yaml` 与 launch 文件配置，避免硬编码。
