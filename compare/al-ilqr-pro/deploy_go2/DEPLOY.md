# AL-iLQR Go2 真机部署与测试手册

> 目标平台：宇树 Go2 内置 Jetson NX（Ubuntu 20.04 + ROS1 Noetic）
> 狗 IP：`192.168.123.18` | 用户：`unitree` | 密码：`123` | 工程目录：`/home/unitree/code/OAYN`
> 本手册是人机协作的执行脚本：按 Phase 顺序逐步执行，每步可验证、可打勾。
> **未完成上一 Phase 的全部通过标准，不要进入下一 Phase。**

## 概述

AL-iLQR Go2 是基于增广拉格朗日 iLQR 的局部轨迹优化器，适配宇树 Go2 四足机器人。
它接收全局路径引导（来自 Odin map_planner），实时优化局部轨迹并输出 cmd_vel 控制指令。

### 系统架构

```
Odin-Nav-Stack（已部署）          al_ilqr_go2（本包）
┌─────────────────────┐          ┌──────────────────────┐
│ odin_ros_driver     │──odom──→│                      │
│ map_planner         │──path──→│  iLQR 求解器         │
│ pc_to_scan(/scan)   │──scan──→│  + 动态障碍约束      │
│                     │          │  + bicycleToDiff     │
│ unitree_control  ←──│──────────│──cmd_vel             │
└─────────────────────┘          └──────────────────────┘
```

关键话题链路：

| 话题 | 方向 | 说明 |
|------|------|------|
| `/odin1/odometry` | Odin → 本包 | 里程计（≥10Hz） |
| `/initial_path` | map_planner → 本包 | 全局 A* 路径 |
| `/scan` | pc_to_scan → 本包 | 前向 90° 激光（避障源，默认） |
| `/cmd_vel` | 本包 → unitree_control | 差速控制指令 |
| `/al_ilqr/optimized_path` | 本包 → RViz | 优化轨迹（硬编码发布） |
| `/al_ilqr/status` | 本包 → 监控 | 求解状态与统计（硬编码发布） |
| `/neupan/arrive` | 本包 → goal_state_machine | 到达事件（std_msgs/Empty） |

### 测试路线图

```
Phase 0 安全与准备 → Phase 1 环境检查 → Phase 2 部署编译
→ Phase 3 静态验证（不动） → Phase 4 车体运动测试（低速）
→ Phase 5 集成导航测试（由易到难） → Phase 6 参数调优
```

---

# Phase 0：安全规程与准备

## 0.1 安全须知（开始任何测试前必读）

### 急停方式（按优先级从高到低）

| 紧急程度 | 操作 | 说明 |
|---|---|---|
| 高（即将碰撞） | Go2 遥控器 **L1+L2 长按** | 硬急停，SDK 层接管，优先级最高 |
| 中（快速偏转） | Ctrl-C 杀掉 al_ilqr_go2 节点，再发零速度 | `rosnode kill /al_ilqr_go2_node` |
| 低（缓慢偏航） | 持续发布零速度覆盖 | 见下方命令 |

零速度覆盖命令（新开终端随时待命）：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist "{}" -r 10
```

> 注意：零速度覆盖与规划器输出会竞争 `/cmd_vel`，只适合规划器已停/已杀的场景。
> 真正紧急时**永远优先用遥控器 L1+L2**。

### 测试场地要求

- [ ] 开阔平整场地（≥ 5m × 5m），无台阶、无松动线缆
- [ ] 周围无易碎物、无玻璃门、无人员密集区
- [ ] 全程至少 1 人手持遥控器监护，随时准备急停
- [ ] 狗电量 > 60%（一轮完整测试约 30-60 分钟）

### 速度原则

- 一律从低速开始（Phase 4 强制限速 0.3 m/s），每轮测试通过后才逐级提速
- 每次只调一个参数，改完立刻记录（见附录 B 测试记录表）

## 0.2 物料准备清单

- [ ] **重定位地图**（`.bin`）：odin driver 启动时加载，提供 map→odom→base TF
- [ ] **栅格地图**（`.pgm` + `.yaml`）：map_server 加载，须与 `.bin` 为同一场景
- [ ] **RViz PC**：与 NX 同一网络（WiFi `MT_dog`，密码 `MT_Dev888`），已装 ROS Noetic
- [ ] **Go2 遥控器**：已开机、已连接
- [ ] 本部署包 `deploy_go2/` 源码（在开发机上）

### RViz PC 的 ROS 网络配置

```bash
# PC 上（每个终端都要）
export ROS_MASTER_URI=http://192.168.123.18:11311
export ROS_IP=$(hostname -I | awk '{print $1}')   # 你 PC 的 WiFi IP
rviz
```

验证连通：

```bash
ping 192.168.123.18          # 应有回应
rostopic list                # 应能看到狗上的话题
```

> 若担心 WiFi 中断影响 ROS：狗上所有终端使用单机配置
> `export ROS_MASTER_URI=http://localhost:11311; export ROS_HOSTNAME=localhost; export ROS_IP=127.0.0.1`，
> 此时 PC 端 RViz 将无法连接，可改用 Foxglove（`ws://192.168.123.18:8765`，
> 需 whole.launch 已启动）。二者选其一，测试前确定网络方案。

### NX 上的终端布局（建议 5 个，均 `ssh unitree@192.168.123.18`）

```
Terminal 1 — odin_ros_driver
Terminal 2 — map_planner whole.launch（含 map_server / fake360 / pc_to_scan）
Terminal 3 — al_ilqr_go2
Terminal 4 — unitree_control（Phase 4 起才启动）
Terminal 5 — 监控 / 录 bag / 急停待命
```

狗上每个终端的通用前置命令：

```bash
cd /home/unitree/code/OAYN
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_IP=127.0.0.1
source ros_ws/devel/setup.zsh
```

> 若用 PC RViz 联调，把上面三行 export 替换为
> `export ROS_MASTER_URI=http://192.168.123.18:11311; export ROS_IP=192.168.123.18`。

---

# Phase 1：环境检查（NX 上）

**前置条件**：已 ssh 进 NX。

## 操作步骤

逐项执行检查命令，比对预期输出：

| # | 检查项 | 命令 | 预期输出 |
|---|--------|------|----------|
| 1 | Ubuntu 版本 | `lsb_release -a` | `Ubuntu 20.04.x LTS` |
| 2 | ROS 版本 | `rosversion -d` | `noetic` |
| 3 | Odin workspace | `ls /home/unitree/code/OAYN/ros_ws/devel/setup.zsh` | 文件存在 |
| 4 | map_planner 包 | `source ros_ws/devel/setup.zsh && rospack find map_planner` | 输出包路径 |
| 5 | odin_ros_driver 包 | `rospack find odin_ros_driver` | 输出包路径 |
| 6 | unitree_control 包 | `rospack find unitree_control` | 输出包路径 |
| 7 | Eigen3 | `dpkg -l libeigen3-dev \| grep ii` | `ii libeigen3-dev ...` |
| 8 | PCL | `dpkg -l libpcl-dev \| grep ii` | `ii libpcl-dev ...` |
| 9 | catkin 工具 | `which catkin` | 输出路径（catkin_tools） |
| 10 | 磁盘空间 | `df -h /home` | 可用 ≥ 2GB |
| 11 | 内存 | `free -h` | 可用 ≥ 2GB（编译需要） |
| 12 | 系统时间 | `date` | 与当前实际时间一致，偏差大则 `sudo ntpdate ntp.aliyun.com` |

> 第 7/8 项若未安装，Phase 2 的 `install_deps.sh` 会自动安装，不算失败。
> 第 9 项若无 catkin_tools，可用 `catkin_make` 替代（Phase 2 有说明）。

## 通过标准

- [ ] Ubuntu 20.04 + ROS Noetic 确认
- [ ] `rospack find` 能找到 map_planner / odin_ros_driver / unitree_control
- [ ] 磁盘可用 ≥ 2GB，内存可用 ≥ 2GB
- [ ] 系统时间正确

## 失败处理

- `rospack find` 失败 → 未 source workspace，或 Odin-Nav-Stack 未部署完整，先解决再继续
- 磁盘不足 → 清理 `~/.ros/log`（`rosclean purge -y`）和旧 bag 文件

---

# Phase 2：部署与编译

**前置条件**：Phase 1 全部通过。

## 2.1 拷贝部署包到 NX

在**开发机**上执行（二选一）：

```bash
# 方式 A：rsync（推荐，可增量重传）
rsync -avz --exclude build/ \
  /home/yu/article_and_warehouse/algo-warehouse/compare/al-ilqr-pro/deploy_go2/ \
  unitree@192.168.123.18:/home/unitree/code/OAYN/ros_ws/src/al_ilqr_go2/

# 方式 B：scp
scp -r /home/yu/article_and_warehouse/algo-warehouse/compare/al-ilqr-pro/deploy_go2 \
  unitree@192.168.123.18:/home/unitree/code/OAYN/ros_ws/src/al_ilqr_go2
```

验证（NX 上）：

```bash
ls /home/unitree/code/OAYN/ros_ws/src/al_ilqr_go2
# 应见：CMakeLists.txt package.xml config/ include/ launch/ scripts/ src/
```

## 2.2 安装依赖（NX 上）

```bash
cd /home/unitree/code/OAYN/ros_ws/src/al_ilqr_go2
source /opt/ros/noetic/setup.bash
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh
```

预期：脚本末尾输出"依赖安装完成！"。

## 2.3 编译

```bash
cd /home/unitree/code/OAYN/ros_ws
catkin build al_ilqr_go2
source devel/setup.zsh
```

- NX 内存不足（编译卡死 / 报 `c++: fatal error: Killed`）时用单线程：

```bash
catkin build al_ilqr_go2 -j1
```

- 若 workspace 用的是 catkin_make：

```bash
cd /home/unitree/code/OAYN/ros_ws
catkin_make --only-pkg-with-deps al_ilqr_go2 -j1
```

验证：

```bash
rospack find al_ilqr_go2                       # 输出包路径
rosrun al_ilqr_go2 al_ilqr_go2_node --help 2>&1 | head -1   # 可执行文件存在（会因无 master 报错，属正常）
ls devel/lib/al_ilqr_go2/al_ilqr_go2_node      # 二进制存在
```

## 2.4 Odin-Nav-Stack 侧的三处修改确认（重要）

本包依赖 Odin 栈的以下修改，部署时逐一确认：

1. **pc_to_scan 高度过滤**（whole.launch 中 pointcloud_to_laserscan 节点）：
   `min_height=-0.2`、`max_height=0.4`（相对 odin1_base_link）。
   若仍为旧值 `0.0`/`0.3`，会漏掉传感器下方（机器人身体高度）的障碍物。

   ```bash
   grep -n "min_height\|max_height" $(rospack find map_planner)/launch/whole.launch
   ```

2. **fake360 衰减时间**：`fake360_cell_decay_sec=6.0`（whole.launch）。

   ```bash
   grep -n "fake360_cell_decay_sec" $(rospack find map_planner)/launch/whole.launch
   ```

3. **map_planner.cpp 范围外报错增强**：目标点超出地图范围时输出
   `Goal (x, y) is outside the map bounds` 警告。若此修改是新同步的，
   **需要重新编译 map_planner**：

   ```bash
   cd /home/unitree/code/OAYN/ros_ws && catkin build map_planner -j1
   ```

## 通过标准

- [ ] `al_ilqr_go2` 编译零错误，`devel/lib/al_ilqr_go2/al_ilqr_go2_node` 存在
- [ ] `rospack find al_ilqr_go2` 正常
- [ ] whole.launch 中 `min_height=-0.2`、`max_height=0.4` 已确认
- [ ] `fake360_cell_decay_sec=6.0` 已确认
- [ ] map_planner 范围外报错修改已同步并重新编译（如适用）

## 失败处理

- Eigen 头文件找不到 → `sudo apt-get install libeigen3-dev`，确认 `/usr/include/eigen3` 存在
- PCL 链接错误 → 确认 `ros-noetic-pcl-ros` 已装（install_deps.sh 包含）
- 编译进程被杀 → 用 `-j1`；仍不行则 `sudo swapon`（检查 swap 是否启用）

---

# Phase 3：静态验证（机器人趴地/悬空，不动）

**前置条件**：Phase 2 通过。**本阶段全程不启动 unitree_control**，
`/cmd_vel` 有输出但不会驱动机器人，绝对安全。机器人趴地或悬空放置。

## 3.1 启动 odin driver 并验证里程计 + TF

Terminal 1：

```bash
cd /home/unitree/code/OAYN && source ros_ws/devel/setup.zsh
roslaunch odin_ros_driver odin1_ros1.launch
```

> 启动后需重定位：用遥控器控狗小幅前后左右移动（悬空测试可跳过，
> 但集成测试前必须完成重定位）。

Terminal 5 验证：

```bash
rostopic hz /odin1/odometry        # 预期 ≥ 10Hz，稳定
rostopic hz /odin1/cloud_slam      # 预期有稳定输出（点云）
rosrun tf tf_echo map odin1_base_link   # 预期连续输出变换，无 "cannot find" 报错
rosrun tf tf_echo map odom              # 必预期有输出：/initial_path 在 map 系，
                                        #  本包需要 map→odom 把路径变换到 odom 系
rostopic echo -n1 /initial_path/header/frame_id  # 确认路径坐标系（通常 "map"）
rosrun tf view_frames && evince frames.pdf   # 可选：TF 树应含 map→odom→odin1_base_link
```

- [ ] `/odin1/odometry` ≥ 10Hz
- [ ] `/odin1/cloud_slam` 有数据
- [ ] TF 链 map→odom→odin1_base_link 完整

## 3.2 启动 whole.launch 并验证 /scan

Terminal 2：

```bash
cd /home/unitree/code/OAYN && source ros_ws/devel/setup.zsh
roslaunch map_planner whole.launch
```

Terminal 5 验证：

```bash
rostopic hz /scan                                  # 预期与点云频率同级，稳定
rostopic echo -n1 /scan/header/frame_id            # 预期 "odin1_base_link"
rostopic hz /scan_360                              # fake360 输出，应有数据
rostopic echo -n1 /map/info                        # map_server 已加载 pgm
```

RViz 中确认高度过滤效果（添加 LaserScan display，topic `/scan`，Fixed Frame `map`）：

- 人站在传感器正前方 1-2m → `/scan` 中**应出现**对应障碍点
- 空旷地面方向 → **不应**出现成片的地面点（若地面成为障碍，说明
  min_height 仍是旧值，回 Phase 2.4）

- [ ] `/scan` 有稳定输出，frame_id = `odin1_base_link`
- [ ] 人在传感器前站立可见
- [ ] 地面不被当作障碍
- [ ] `/scan_360`、`/map` 均有数据

## 3.3 验证全局规划（map_planner）

RViz 中点击 **2D Nav Goal**，在地图空旷处选一个目标点。

Terminal 5 验证：

```bash
rostopic echo -n1 /move_base_simple/goal     # 确认目标点已发出
rostopic echo -n1 /initial_path | head -20   # 应有 poses 数组
```

RViz 添加 Path display（topic `/initial_path`）：应显示从机器人当前位置到目标的路径。

- [ ] 发目标点后 `/initial_path` 立即生成
- [ ] 路径起点在机器人当前位置附近
- [ ] 故意点地图范围外的目标 → map_planner 输出 `Goal (x, y) is outside the map bounds` 警告且不生成路径

## 3.4 启动 al_ilqr_go2 并验证求解输出

Terminal 3：

```bash
cd /home/unitree/code/OAYN && source ros_ws/devel/setup.zsh
roslaunch al_ilqr_go2 al_ilqr_go2.launch
```

再从 RViz 发一个目标点，Terminal 5 逐项验证：

```bash
rosnode info /al_ilqr_go2_node          # 订阅/发布话题连接正常
rostopic echo /al_ilqr/status           # 见下方状态说明
rostopic hz /al_ilqr/optimized_path     # 预期 ~10Hz（planning/rate）
rostopic hz /cmd_vel                    # 预期 ~10Hz
rostopic echo /cmd_vel                  # 数值范围检查，见下
```

状态机含义（`/al_ilqr/status`）：

| 状态 | 含义 |
|------|------|
| `WAITING_FOR_ODOM` | 未收到里程计（检查 3.1） |
| `WAITING_FOR_PATH` | 未收到全局路径（检查 3.3） |
| `ODOM_TIMEOUT` | 里程计超过 `planning/odom_timeout`（默认 0.5s）未更新，停车 |
| `INVALID_STATE` | 位姿/速度出现 NaN/inf，停车 |
| `PATH_TF_FAILED` | 全局路径 frame_id → `odom_frame` 的 TF 查询失败，停车 |
| `PATH_TOO_SHORT` | 变换后路径点少于 2 个，停车 |
| `GUIDANCE_TOO_SHORT` | guidance 采样点少于 2 个，停车 |
| `INVALID_CONTROL` | 求解输出/映射结果非有限，停车 |
| `OK` | 正常求解中 |
| `SOLVER_FAILED` | 求解不收敛，输出零速度 |
| `GOAL_REACHED` | 已到达，停车 |

`OK` 状态字段格式：

```
OK | cvg=1 | iter=(1/6) | 8.3ms | analytical | viol=0.021 | obs=3 | min_r=1.84 | a=0.42 | s=-0.07 | v=0.55 | w=-0.10
```

- `analytical` / `numerical`：实际生效的求导模式（来自 `solver/derivative_mode`）
- `min_r`：`/scan` 最近有效距离 m（-1 表示暂无有效点）

`/cmd_vel` 数值合理性检查（静态下机器人不动，但输出应满足约束）：

- `linear.x` ∈ [-0.3, +0.8]（受 `control_mapping/max_linear_vel=0.8` 与 `vehicle/min_speed=-0.3` 限制）
- `angular.z` ∈ [-1.2, +1.2]（受 `control_mapping/max_angular_vel=1.2` 限制）
- 无 NaN、无剧烈跳变（相邻帧差值应平滑）

求解耗时检查：观察 `/al_ilqr/status` 中的 `%.1fms` 字段。启用解析求导后
NX 上典型 **< 15ms**（标称场景 1-3ms，障碍密集且跑满 4×12 迭代的最坏情况
约 10-15ms）。若显示 `numerical` 且耗时 ~120ms，说明 `solver/derivative_mode`
未生效，检查 yaml 是否被 launch 覆盖。

- [ ] `/al_ilqr/status` 为 `OK`
- [ ] `/al_ilqr/optimized_path` 以 ~10Hz 发布，RViz 中显示为贴合全局路径的局部轨迹
- [ ] `/cmd_vel` 数值在限幅内、无 NaN
- [ ] 求解耗时 < 15ms（且状态字段显示 `analytical`）
- [ ] `/al_ilqr/optimized_path` 的 frame_id = `odom`，且与 `/initial_path`（map 系）
      在 RViz 中重合——若两者相差一个固定平移，说明 map→odom 变换异常

## 3.5 RViz 可视化配置（后续 Phase 通用）

添加以下 Display：

- Path：`/initial_path`（全局路径，建议蓝色）
- Path：`/al_ilqr/optimized_path`（优化轨迹，建议绿色）
- LaserScan：`/scan`（避障源）
- Map：`/map`
- Odometry：`/odin1/odometry`
- TF

## 失败处理

- 状态卡在 `WAITING_FOR_ODOM` → `rostopic hz /odin1/odometry`；若话题名不同，
  用 launch 参数覆盖：`roslaunch al_ilqr_go2 al_ilqr_go2.launch odometry_topic:=<实际话题>`
- 状态卡在 `WAITING_FOR_PATH` → 确认 RViz 已发目标且 `/initial_path` 有数据
- `SOLVER_FAILED` 持续 → 见 Phase 7"求解不收敛"

---

# Phase 4：车体运动能力测试（接入 unitree_control，低速）

**前置条件**：Phase 3 全部通过；机器人站立于开阔场地；监护人手持遥控器。

## 4.0 先限速（必做）

编辑 `config/al_ilqr_go2.yaml`，将输出限幅调低：

```yaml
control_mapping:
  virtual_wheelbase: 0.4
  max_linear_vel: 0.3       # 原 0.8，测试期间限低
  max_angular_vel: 0.5      # 原 1.2，测试期间限低
```

> 改完 yaml 必须重启 al_ilqr_go2 节点才生效（Ctrl-C 后重新 roslaunch）。
> 本 Phase 的 4.1-4.4 为手动发 cmd_vel 测试，**期间不要运行 al_ilqr_go2**
> （避免与手动指令竞争 /cmd_vel）；限速修改是为 Phase 5 做准备，现在改好。

Terminal 4 启动底盘控制（unitree_control 订阅 /cmd_vel 驱动 Go2 SDK）：

```bash
cd /home/unitree/code/OAYN && source ros_ws/devel/setup.zsh
# 按 Odin-Nav-Stack 现场部署方式启动 unitree_control 节点
rosrun unitree_control unitree_control    # 若实际为 launch 文件请按现场为准
```

启动后确认：`rosnode list | grep unitree` 有输出。

## 4.1 手动 cmd_vel 直线测试

**操作**：在地面标记起点，发固定速度走约 2m（0.2 m/s × 10s）：

```bash
# 记录起点 odom 读数
rostopic echo -n1 /odin1/odometry/pose/pose/position

# 发直线指令（10 秒后 Ctrl-C，随后立刻发零速度）
timeout 10 rostopic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}}" -r 10
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"

# 记录终点 odom 读数
rostopic echo -n1 /odin1/odometry/pose/pose/position
```

**记录表**：

| 项目 | 期望值 | 实测值 | 偏差 |
|------|--------|--------|------|
| 卷尺实测直线距离 | 2.0 m | | |
| odom 位移（终点-起点欧氏距离） | 2.0 m | | |
| 横向偏移（偏离直线） | < 0.2 m | | |

- [ ] 实际行走 ≈ 2m（偏差 < 10%）
- [ ] odom 读数与卷尺实测一致（偏差 < 10%）
- [ ] 直线性良好（横向漂移 < 0.2m）

## 4.2 手动 cmd_vel 原地旋转测试

**操作**：angular.z = 0.3 rad/s，转约 10s（理论转 3 rad ≈ 172°）：

```bash
timeout 10 rostopic pub /cmd_vel geometry_msgs/Twist \
  "{angular: {z: 0.3}}" -r 10
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"

# 期间另开终端观察实际角速度
rostopic echo /odin1/odometry/twist/twist/angular/z
```

**记录表**：

| 项目 | 期望值 | 实测值 | 偏差 |
|------|--------|--------|------|
| 旋转方向 | 逆时针（z>0 为左转） | | |
| odom 角速度均值 | 0.3 rad/s | | |
| 目测旋转角度（10s） | ≈ 172° | | |

- [ ] 正 angular.z 对应逆时针（左转），方向正确
- [ ] odom 角速度与指令一致（偏差 < 20%，超出则 4.4 标定修正）

## 4.3 急停测试

逐一验证三种急停方式，每种测试时先让机器人以 0.2 m/s 直行：

```bash
# 启动直行（后台）
rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" -r 10 &
```

| 方式 | 操作 | 通过标准 |
|------|------|----------|
| 零速度覆盖 | 杀掉上面的 pub（`kill %1`），立刻 `rostopic pub /cmd_vel geometry_msgs/Twist "{}" -r 10` | 1s 内停止 |
| 杀节点 | 直接 `kill %1` 停止发布 | unitree_control 超时保护应使机器人停止 |
| 遥控器接管 | 直行中长按 **L1+L2** | 立即趴下/锁定，cmd_vel 不再生效 |

- [ ] 零速度覆盖：1s 内停止
- [ ] 停止发布 cmd_vel：机器人停止（确认超时保护存在；若持续行走为严重隐患，立即遥控器急停并排查 unitree_control）
- [ ] 遥控器 L1+L2：立即硬急停，且急停后手动 cmd_vel 无法驱动（需遥控器恢复）

## 4.4 虚拟轴距标定

`virtual_wheelbase` 决定自行车模型→差速映射的转向精度，是最关键的标定项。

**自动标定**（推荐）：

```bash
cd /home/unitree/code/OAYN/ros_ws/src/al_ilqr_go2
python3 scripts/calibrate_wheelbase.py
# 脚本流程：发 angular.z=0.5 原地转 10s → 统计 odom 实际角速度 → 输出建议值
```

脚本默认参数：`test_angular_vel=0.5`、`test_duration=10.0`、`odom_topic=/odin1/odometry`，
可用 ROS 私有参数覆盖（如 `_test_angular_vel:=0.3`）。

**记录表**：

| 项目 | 值 |
|------|-----|
| 期望角速度 | 0.5 rad/s |
| 实际角速度（均值 ± 标准差） | |
| 比值（实际/期望） | |
| 当前 virtual_wheelbase | 0.4 |
| 建议 virtual_wheelbase | |

将建议值写入 `config/al_ilqr_go2.yaml`：

```yaml
control_mapping:
  virtual_wheelbase: <建议值>
```

**手动标定**（备用）：

1. 设 `virtual_wheelbase = 0.4`（初始值）
2. 发 `cmd_vel: linear.x=0.3, angular.z=0.5`，期望转弯半径 R_expected = 0.3/0.5 = 0.6m
3. 目测/卷尺量实际转弯半径 R_actual
4. 修正：`virtual_wheelbase_new = virtual_wheelbase × R_actual / R_expected`

- [ ] 标定脚本正常完成，数据点充足
- [ ] 建议值已写入 yaml 并记录在测试记录表

## 失败处理

- cmd_vel 有值但狗不动 → `rosnode list` 确认 unitree_control 在跑；
  `rostopic info /cmd_vel` 确认其为订阅者
- 标定脚本报"实际角速度接近零" → 机器人未真正运动（unitree_control 未工作）
  或里程计异常（回 3.1）
- 直线漂移大 → 检查场地是否平整；漂移持续存在记录下来，Phase 5 观察闭环下是否被纠正

---

# Phase 5：集成导航测试（由易到难）

**前置条件**：Phase 4 全部通过；yaml 限速仍为低速（0.3/0.5）；
五个节点全部运行（odin driver、whole.launch、unitree_control、al_ilqr_go2）；
已完成重定位（遥控器控狗小幅移动直至定位稳定）。

**每个场景开始前建议录 bag**（见附录 A），便于回放分析。

## 5.1 短距离直线给点（3-5m 无障碍）

**操作**：RViz 2D Nav Goal 点选机器人正前方 3-5m 空旷处。

**RViz 观察要点**：`/initial_path`（蓝）生成 → `/al_ilqr/optimized_path`（绿）
贴合全局路径 → 机器人平滑跟踪 → 接近终点自然减速。

**通过标准**：

- [ ] 机器人自动出发，全程平滑无急停急转
- [ ] 到达后停车：`/al_ilqr/status` 显示 `GOAL_REACHED`
- [ ] 停车位置距目标 < 0.3m（`goal/position_tolerance`）
- [ ] 无终点振荡（不冲过终点来回修正）
- [ ] `/neupan/arrive` 收到一次到达事件：`rostopic echo /neupan/arrive`（同一路径只发一次）

## 5.2 静态障碍绕行

**布置**：5.1 同样的直线路径，中间放一个纸箱（约 0.3×0.3m），
两侧留 ≥ 0.7m 通道。

**操作**：发同样的目标点。

**RViz 观察要点**：

- `/scan` 中纸箱出现为聚类点
- `/al_ilqr/optimized_path` 在纸箱附近向侧面偏移（局部避障）
- 若全局路径被 fake360 记忆地图更新后重新规划，`/initial_path` 也会绕开

**通过标准**：

- [ ] 机器人绕开纸箱，不碰撞、不擦边（最近距离 ≥ `dynamic_obstacles/inflate_radius` 0.3m 量级）
- [ ] 绕行后回到全局路径继续前进
- [ ] 正常到达终点 `GOAL_REACHED`

## 5.3 动态障碍测试

**需配合人员 1 名。监护人保持急停待命。**

**5.3a 横穿**：机器人直线导航中，人从侧面以步行速度走入前方路径（1-2m 处）再走开。

- [ ] 机器人减速或局部绕行，不碰撞
- [ ] 人离开后恢复原速继续导航并到达

**5.3b 挡死**：人站定在路径正中不动，完全挡住去路。

- [ ] 机器人减速并停车等待（绕行无解时状态转 `SOLVER_FAILED`，输出零速度）
- [ ] 人让开后自动恢复行进（无需人工干预）
- [ ] 全程无碰撞、无撞人趋势

**观察要点**：`rostopic echo /al_ilqr/status` 看状态切换；
RViz 中 `/scan` 的人体聚类与 `/al_ilqr/optimized_path` 的形变。

> 注意：`/scan` 只有前向 90° FOV，人从正后方接近属于盲区，为预期行为。
> 需要后方感知时切 `/scan_360`（见附录 C），并注意幽灵障碍问题。

## 5.4 长距离多弯路径

**操作**：给一个需要转 ≥ 2 个弯的远目标（10m+，视场地）。

- [ ] 全程跟踪全局路径，转弯半径合理（过大→Phase 6 调 virtual_wheelbase）
- [ ] 无中途卡死、无振荡
- [ ] 到达 `GOAL_REACHED`，停车误差 < 0.3m
- [ ] 观察求解耗时是否稳定 < 15ms（长路径下 guidance 前视更远）

## 5.5 提速验证（可选，逐级）

5.1-5.4 全部通过后，将 `control_mapping/max_linear_vel` 逐级恢复
0.3 → 0.5 → 0.8（`max_angular_vel` 相应 0.5 → 0.8 → 1.2），
每级重跑 5.1 和 5.2，通过后再升下一级。

- [ ] 0.5 m/s 级通过
- [ ] 0.8 m/s 级通过

## 失败处理

- 发目标点后不动 → 依序查：`/initial_path` 有无数据 → `/al_ilqr/status`
  状态 → `/cmd_vel` 有无输出 → unitree_control 是否在跑
- 到达被截断路径终点后不再前进 → 确认 goal_state_machine 在跑且
  `goal/arrive_topic=/neupan/arrive` 一致（自动重规划链路，见 Phase 7）
- 其余见 Phase 7

---

# Phase 6：参数调试指南

所有参数在 `config/al_ilqr_go2.yaml`。**每次只调一个参数**，修改后必须
重启 al_ilqr_go2 节点（Ctrl-C → 重新 roslaunch）才生效，并记录到测试记录表。

## 6.1 问题现象 → 参数速查表

| 现象 | 调整参数 | 方向 |
|------|----------|------|
| 转弯半径偏大/转向迟钝 | `control_mapping/virtual_wheelbase` | 重新标定（Phase 4.4），或按比例减小 |
| 避障太保守、绕得太远 | `dynamic_obstacles/inflate_radius` | 减小（不低于 0.2） |
| 避障擦边、距障碍太近 | 同上 | 增大 |
| 速度/转向抖动 | `weights/accel_reg_weight`、`weights/steering_reg_weight` | 增大（平滑项） |
| 求解超时（>50ms） | `dynamic_obstacles/max_obstacles`、`solver/horizon`、`solver/max_inner_iterations` | 减小；或 `solver/dt` 0.1→0.15 |
| 路径跟踪偏差大 | `weights/position_weight` | 增大 |
| 终点停不准 | `weights/terminal_position_weight` | 增大 |
| 速度忽快忽慢 | `weights/speed_weight` | 增大 |
| 一个障碍被识别成多个 | `dynamic_obstacles/cluster_tolerance` | 增大 |
| 多个障碍被合并成一个大障碍 | `dynamic_obstacles/cluster_tolerance` | 减小 |
| 行走太慢 | `planning/ref_speed`、`vehicle/max_speed`、`control_mapping/max_linear_vel` | 同步增大 |
| 求解频繁不收敛 | `solver/max_inner_iterations` 增大，或速度限制减小 | |

## 6.2 速度与安全参数

| 参数 | 保守值 | 正常值 | 激进值 |
|------|--------|--------|--------|
| `vehicle/max_speed` | 0.5 | 0.8 | 1.2 |
| `control_mapping/max_linear_vel` | 0.5 | 0.8 | 1.0 |
| `control_mapping/max_angular_vel` | 0.8 | 1.2 | 1.5 |
| `vehicle/max_acceleration` | 0.5 | 1.0 | 1.5 |
| `planning/ref_speed` | 0.5 | 0.8 | 1.0 |

> `path/max_speed` 与 `vehicle/max_speed` 建议保持一致。

## 6.3 求解器性能（NX 上）

- `solver/derivative_mode: "analytical"`（**必须保持**）。解析求导 vs 中心差分
  实测差 10x 以上：x86 参考机（horizon=20、10 个障碍、跑满 4×12 迭代）
  80.5ms → 6.2ms，两种模式轨迹一致（max|Δu| ≈ 3e-6）。
  该参数在 Task #13 之前只写在 yaml、C++ 未读取，因此实际一直跑中心差分
  （NX ~124ms/次）；现已由 `loadParameters()` 读取并传入求解器。
- `solver/horizon: 15-25`（当前 20；越小越快，越大预测越远）
- `solver/max_inner_iterations: 8-15`（当前 12；越小越快，但可能不收敛）
- `solver/convergence_threshold: 1.0e-3`（内层代价收敛阈值；增大可提前退出）
- `dynamic_obstacles/max_obstacles: 10`（**耗时主因之一**：每个障碍在每个 stage
  各加一条约束，10 个障碍 × 20 步 = 200 条约束。若仍超时优先降到 5-6）

如果求解频率 < 10Hz（即单次 > 100ms），按序排查/尝试：

0. 先确认 `/al_ilqr/status` 中显示的是 `analytical`（不是 `numerical`）
1. 减小 `dynamic_obstacles/max_obstacles`（10 → 6）
2. 减小 `horizon`（20 → 15）
3. 减小 `max_inner_iterations`（12 → 8）
4. 增大 `dt`（0.1 → 0.15，预测窗口不变但步数少）

## 6.4 轨迹质量权重

- 路径偏差大 → 增大 `weights/position_weight`（当前 10.0）
- 转向不平滑 → 增大 `weights/steering_reg_weight`（当前 1.0）
- 速度波动 → 增大 `weights/speed_weight`（当前 2.0）
- 终点不准 → 增大 `weights/terminal_position_weight`（当前 50.0）
- 机器人抖动 → 增大 `weights/accel_reg_weight` 和 `weights/steering_reg_weight`；
  也可降低 `planning/rate`（10 → 5 Hz）

## 6.5 注意事项

- 障碍物参数已全部收敛到 `dynamic_obstacles` 段（`enable / scan_topic /
  inflate_radius / max_obstacles / cluster_tolerance / max_range /
  min_radius / max_radius`），旧的 `obstacle` 段（多数键从未被 C++ 读取）
  已从 yaml 中删除
- `/initial_path` 通常发布在 **map** 系，节点会按最新 TF（`map -> odom_frame`）
  自动变换到 `odom_frame` 后再送入优化器；若 TF 链断裂状态会显示
  `PATH_TF_FAILED` 并停车（必须保证 SLAM 在发布 map→odom）
- `planning/enable_road_boundary` 保持 `false`：该约束是 odom 系 y 向固定
  走廊而非真实车道线，开启后机器人走出 y∈[road.lower_bound, road.upper_bound]
  会导致约束失败
- `goal/check_heading` 默认 `false`：全局路径终点的 orientation 由 map_planner
  生成，强制对齐容易导致到达后原地转圈；确有停车朝向要求时才置 true
- `goal/arrive_topic` 必须保持 `/neupan/arrive`（goal_state_machine 订阅），
  否则到达后自动重规划链路断裂

---

# Phase 7：故障排查（按现象分类）

## 7.1 机器人一动不动

按链路顺序排查：

1. 全局路径：`rostopic echo /initial_path | head`——无数据则 map_planner
   未收到 goal，检查 RViz 发的是 `/move_base_simple/goal`
2. 里程计：`rostopic hz /odin1/odometry`（应 ≥ 10Hz）
3. 规划器状态：`rostopic echo /al_ilqr/status`
   - `WAITING_FOR_ODOM`：里程计未收到 → 检查 odin driver / 话题名
   - `WAITING_FOR_PATH`：全局路径未收到 → 检查 map_planner
   - `SOLVER_FAILED`：求解不收敛 → 见 7.2
4. 控制输出：`rostopic echo /cmd_vel`——有值但狗不动 → unitree_control
   未运行或未订阅（`rostopic info /cmd_vel` 查订阅者）

## 7.2 求解不收敛（SOLVER_FAILED）

- 降低速度限制（`max_speed` / `ref_speed`）
- 减小 `horizon`
- 检查初始状态是否合理（odom 是否跳变？`rostopic echo /odin1/odometry` 观察位姿连续性）
- 增大 `max_inner_iterations`
- 前方被完全堵死时 SOLVER_FAILED 属预期行为（停车等待，障碍移开自动恢复）

## 7.3 机器人抖动

- 降低 `planning/rate`（10 → 5 Hz）
- 增大 `control_mapping/max_linear_vel` / `max_angular_vel` 限幅（减少输出饱和削顶）
- 增大 `weights/accel_reg_weight` 和 `weights/steering_reg_weight`（平滑控制）

## 7.4 避障不工作

- `rostopic hz /scan` 检查激光有无数据
- `rostopic hz /odin1/cloud_slam` 检查上游点云（pc_to_scan 的输入）
- 确认 whole.launch 中 pc_to_scan 高度过滤为 `min_height=-0.2`、`max_height=0.4`
- 确认 yaml 中 `dynamic_obstacles/enable: true`
- 调整 `cluster_tolerance`（太小→一个障碍被拆成多个）
- 调整 `inflate_radius`（太小→擦边）
- **注意**：`/scan` 仅前向 90° FOV，侧后方障碍不可见属预期行为；
  需 360° 感知切 `scan_topic:=/scan_360`（附录 C）

## 7.5 到达后不触发重规划 / 中途"假到达"后停住

端到端到达链路：距路径终点 < `goal/position_tolerance`(0.3m) → 停车、
状态 `GOAL_REACHED` → 向 `/neupan/arrive` 发一次 Empty → goal_state_machine：

- 距真实目标 ≤ goal_tolerance → 真到达，不重规划
- 距真实目标 > goal_tolerance（路径被局部避障截断时会中途"到达"截断终点）
  → 自动调 plan 服务，用最新 fake_map 快照重新 A* 继续前进
- `enable_arrive_replan=true`（默认）时每次 arrive 都可触发重规划（级联）；
  false 时每个目标只重规划一次

排查：

- `rostopic echo /neupan/arrive` 确认事件发出
- 确认 goal_state_machine 节点在跑
- 确认 `goal/arrive_topic` 与其订阅话题一致（默认 `/neupan/arrive`）

## 7.6 目标点被拒绝

- map_planner 输出 `Goal (x, y) is outside the map bounds` → 目标超出
  `.pgm` 地图范围，重选地图内目标
- 起点（机器人当前位置）也必须在地图内；重定位漂移出地图后需检查定位
  与地图原点配置
- 室外使用需预先建好覆盖目标区域的地图（`.bin` 重定位地图 + 对应 `.pgm`），
  未建图区域无法给点导航

## 7.7 幽灵障碍（仅用 /scan_360 时）

行人走过后机器人对着残留障碍停止：将 whole.launch 的
`fake360_cell_decay_sec` 从 6.0 降到 2.0-2.5s。权衡见附录 C。

---

# 附录

## 附录 A：常用命令速查

### 话题监控

```bash
rostopic hz /odin1/odometry /scan /initial_path /cmd_vel   # 多话题频率
rostopic echo /al_ilqr/status                              # 求解状态
rostopic echo /cmd_vel                                     # 控制输出
rosnode info /al_ilqr_go2_node                             # 节点连接情况
rosrun tf tf_echo map odin1_base_link                      # 定位是否正常
```

### 录 bag（参考 compare/scripts/record_bag.sh）

```bash
# 使用统一录制脚本（若已同步到狗上）
bash record_bag.sh <场景名> al_ilqr 90     # 存到 /home/unitree/code/OAYN/bags/

# 手动录制本包关键话题
rosbag record -O al_ilqr_test \
  /cmd_vel /odin1/odometry /scan /scan_360 /initial_path \
  /al_ilqr/optimized_path /al_ilqr/status \
  /move_base_simple/goal /neupan/arrive /map /tf /tf_static /rosout

# 拉回开发机分析
scp "unitree@192.168.123.18:/home/unitree/code/OAYN/bags/*.bag" ~/bags/
rosbag info ~/bags/xxx.bag
rosbag play ~/bags/xxx.bag --clock -r 1.0   # 配合 RViz 回放
```

### 自定义话题启动

```bash
roslaunch al_ilqr_go2 al_ilqr_go2.launch \
  odometry_topic:=/odin1/odometry \
  global_path_topic:=/initial_path \
  cmd_vel_topic:=/cmd_vel \
  scan_topic:=/scan
```

## 附录 B：测试记录表模板

| 日期 | 测试项（Phase-编号） | 参数版本/改动 | 结果（通过/失败/急停） | bag 文件 | 备注 |
|------|---------------------|---------------|------------------------|----------|------|
| | | | | | |
| | | | | | |

标定记录：

| 日期 | virtual_wheelbase | 标定比值 | 备注 |
|------|-------------------|----------|------|
| | 0.4（初始） | — | |

## 附录 C：/scan ↔ /scan_360 切换指南

### 点云到 /scan 链路

```
/odin1/cloud_slam (PointCloud2, frame=odom)
    → pointcloud_to_laserscan（变换到 odin1_base_link，高度过滤 [-0.2, +0.4]m 相对传感器，90° FOV）
    → /scan (LaserScan, frame_id=odin1_base_link)
```

- **传感器安装**：Odin 传感器安装在 Go2 头部顶部，离地约 0.5m。高度过滤
  下界 -0.2m（以 odin1_base_link 为中心向下 0.2m）对应离地约 0.3m 以上，
  可避免把地面点当作障碍；上界 +0.4m 覆盖传感器上方的悬空障碍。
- **局限性**：`/scan` 只有前向 90° FOV，机器人身后/侧面的障碍不可见。

### 切换机制

监控话题与动态避障话题均由 launch 的 `scan_topic` 一个 arg 控制：
launch 在 `rosparam load` 之后用 `$(arg scan_topic)` 同时覆盖了
`scan_topic` 和 `dynamic_obstacles/scan_topic` 两个参数（动态避障订阅的
话题名来自 `dynamic_obstacles/scan_topic` 参数而非 remap，因此仅靠 remap
不够，必须用 param 覆盖）。用户只改一个 arg 即可完成切换。

```bash
# 默认：/scan（前向 90°，低延迟，无后方感知）
roslaunch al_ilqr_go2 al_ilqr_go2.launch

# 切换：/scan_360（360° 记忆扫描，需 fake360 节点在运行）
roslaunch al_ilqr_go2 al_ilqr_go2.launch scan_topic:=/scan_360
```

### 两者对比

| 特性 | /scan | /scan_360 |
|------|-------|-----------|
| FOV | 前向 90° | 360°（FOV 外为历史记忆） |
| 延迟 | 低（每帧实时） | FOV 内实时；FOV 外有记忆延迟（取决于衰减时间） |
| 幽灵障碍风险 | 无 | 有（已离开的动态障碍在衰减前仍占用栅格） |
| 后方/侧面感知 | 无 | 有（记忆） |
| 适用场景 | 前进为主的常规导航（推荐默认） | 频繁原地转身/倒车、需要后方障碍记忆的狭窄环境 |

### fake360 衰减时间说明

- 当前 `fake360_cell_decay_sec = 6.0`（whole.launch）：后方记忆保持较久。
  默认配置下局部规划器只消费 /scan，衰减长不会引入幽灵障碍问题。
- 如果切换到 /scan_360 作为避障源且出现幽灵障碍卡死（行人走过后机器人
  对着残留障碍停止），建议将其降到 2.0-2.5s。
- 权衡：衰减只影响 FOV 外的栅格（FOV 内每帧刷新）——衰减长 = 后方记忆久
  但残留久；衰减短 = 响应动态环境快但转身后容易丢失后方障碍。

## 附录 D：与 NeuPAN 的对比

| 特性 | AL-iLQR Go2 | NeuPAN |
|------|-------------|--------|
| 规划方法 | 基于优化（iLQR） | 基于势场+优化 |
| 动力学 | 自行车模型+映射 | 直接差速 |
| 预测窗口 | 2.0s (20*0.1) | 实时 |
| 约束处理 | 增广拉格朗日 | 无硬约束 |
| 计算量 | 低（解析求导 ~1-15ms/帧） | 低（~10ms） |
| 轨迹质量 | 高（全局最优） | 中 |
