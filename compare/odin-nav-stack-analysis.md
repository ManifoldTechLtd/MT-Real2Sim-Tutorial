# Odin-Nav-Stack 工程接入分析（方案 C 基底 / 全栈前端）

> 目标：摸清 Odin-Nav-Stack 全栈架构，区分"我们要用的全局前端"vs"它自带的后端 NeuPAN-fork"，明确：
> - 方案 C（NeuPAN 升级版）= 当前栈原样跑；
> - 方案 B（NeuPAN 原版）= 当前栈 + 替换 `NeuPAN/` 子模块为 `hanruihua/NeuPAN`；
> - 方案 D（AL-iLQR）= 借用当前栈的"全局前端 + fake360"，替换 NeuPAN 节点为我们的 AL-iLQR 节点。
> 同步来源：`/home/yu/code_odin_nav_stack/Odin-Nav-Stack/`（本地开发机已有副本，已成功部署在 Go2 上）。

---

## 1. 仓库定位

由 **Manifold Tech Ltd.** 开源，**ROS1 Noetic** 全栈导航系统，围绕 **Odin1 传感器（高频里程计 + 稠密深度 + 点云 + SLAM）+ Unitree Go2** 设计。内含 11 个 ROS 包 + NeuPAN 子模块（fork：`ManifoldTechLtd/NeuPAN @ odin-stack-fixes`）。

> "Reactive local planners combined with global path planning for safe, smooth motion in complicated environments." — `README.md:28-29`

技术栈对应论文级别的"全局 A\* + 局部 NeuPAN (MPC + DUNE)"组合。

---

## 2. ROS 包速查

| 包 | 角色 | 关键节点 | 备注 |
|---|---|---|---|
| `odin_ros_driver` | 传感器驱动 | `odin1_ros1.launch` | 发布 `/odin1/depth_img_competetion`、`/odin1/odometry`、`/odin1/odometry_highfreq`、`/odin1/cloud_slam` 等。README §"Odin1 ROS driver modification" 要求改 `host_sdk_sample.h` 用高频 TF |
| `map_planner` | **全局规划**（核心前端） | `map_planner` + `goal_state_machine` | A* on inflated occupancy grid，**带 4 个增强**：Laplacian 平滑 / heading-aware 起点偏置 / 距离衰减 cost / fake_map 动态障碍融合。`map_planner.cpp:38-69` |
| `fake360` | **障碍记忆层** | `fake360_node` | 从 90° LiDAR /scan 累积成 `/fake_map`（odom 帧 OccupancyGrid）+ 反射 `/scan_360`（360° 虚拟 LaserScan）。给 NeuPAN/A* 提供"转头看不到也记得"的避障能力 |
| `NeuPAN/` (submodule) | **局部规划**（默认后端） | `neupan_node` (python `neupan_core`) | 订 `/scan_360` + `/initial_path` + `/odin1/odometry`，发 `/cmd_vel`。MPC + DUNE 神经几何障碍 |
| `model_planner` | 备选规划 | `global_planner_node` + `local_planner_node` | 纯 A* + DWA。**不与 NeuPAN 同时跑**，是替代后端的备选 |
| `navigation_planner` | 经典 move_base 封装 | move_base | DWA local planner，最传统的方案。也是替代后端 |
| `unitree_control` | 底盘桥 | `unitree_vel_controller` | `/cmd_vel → SportClient.Move`（与 ETH-SRU 同名同源） |
| `pcd2pgm` | 建图工具 | offline | 点云转 occupancy grid 给 `map_server` 用 |
| `pointcloud_saver` | 录点云 | offline | |
| `fish2pinhole` | 图像投影 | cloud_crop | 鱼眼 → pinhole，供 yolo 使用 |
| `yolo_ros` | 目标检测 | yolov5_ros | 语义导航/VLM 支撑 |
| `odin_vlm_terminal` | VLM | 语言对话 | "Go to the chair" 类语义指令 |

辅助脚本：`scripts/str_cmd_control.py`（语音/字符串指令转 goal）、`scripts/VLN.py`（视觉语言导航 demo）、`scripts/map_recording.sh`（建图流程）。

---

## 3. 主栈数据流（whole.launch + NeuPAN）

`map_planner/launch/whole.launch:1-170` 是当前**实机主用 launch**，组装如下：

```
                       map_server
                     (静态地图 PGM)
                            │ /map
                            ▼
   /odin1/cloud_slam ──► pc_to_scan ──► /scan ──► fake360_node ──► /fake_map
                                                          │              │
                                                          ▼              ▼
                                                       /scan_360      /map (融合)
                                                                       │
                            ┌──────────────────────────────────────────┤
                            │                                          │
   /move_base_simple/goal ─►│  map_planner (A*)                        │
                            │  - inflated_map                          │
                            │  - Laplacian 平滑                        │
                            │  - heading-aware 起点偏置                │
                            │  - distance-graduated cost               │
                            │  - 融合 fake_map 动态层                  │
                            └────────────────► /initial_path ──┐       │
                                                                │       │
   /neupan/arrive ────► goal_state_machine ──┐                  │       │
   (replan trigger)                          │                  │       │
                                             └─► (auto re-plan) │       │
                                                                ▼       ▼
                                                          neupan_node (NeuPAN)
                                          /odin1/odometry ─► (TF localisation)
                                          /scan_360 ───────► (障碍点云)
                                                                │
                                                                ▼
                                                            /cmd_vel
                                                                ▼
                                                       unitree_control ─► Go2
```

### 关键话题契约（接 AL-iLQR/NeuPAN 都要遵守）

| 话题 | 类型 | 频率 | 上游 | 下游 |
|---|---|---|---|---|
| `/initial_path` | `nav_msgs/Path` | 触发式 (goal/arrive) | map_planner | NeuPAN / AL-iLQR / DWA |
| `/scan_360` | `sensor_msgs/LaserScan` | ~20 Hz | fake360 | NeuPAN / AL-iLQR |
| `/fake_map` | `nav_msgs/OccupancyGrid` | ~20 Hz | fake360 | map_planner（合并） |
| `/odin1/odometry` | `nav_msgs/Odometry` | ~10 Hz | odin_ros_driver | NeuPAN MPC 反馈 |
| `/cmd_vel` | `geometry_msgs/Twist` | 局部规划频率 | local planner | unitree_control |
| `/neupan/arrive` | `std_msgs/Empty` | 事件 | NeuPAN | goal_state_machine 触发自动 replan |

---

## 4. 全局前端（map_planner）能力定位

`map_planner.cpp:38-69` 的 4 个增强相当于"我们想要的 Hybrid A* 替代方案"的最低门槛：

1. **inflated map**：基础膨胀；
2. **Laplacian smoothing**（`smoothing_iterations=20`）：把 8-connected 路径的 45° 锯齿磨平；
3. **distance-graduated cost**（`obstacle_cost_weight=3.0, safe_distance=0.5`）：让路径走廊居中，**对 NeuPAN/iLQR 都受益**；
4. **heading-aware A\***（`start_heading_penalty_weight=2.0, radius=1.0m`）：起点附近偏向"狗当前 yaw 方向"，避免 replan 后第一段反方向；
5. **fake_map injection**：每次 plan 都把最新动态层喂进去。

这些增强是**为后端 NeuPAN 的 MPC 量身定制**的——给到 AL-iLQR 也直接好用，因为 AL-iLQR 也是基于参考路径的优化。

**Hybrid A\*（执行计划 §后续任务 6）**：如果要替换 `map_planner` 为 Hybrid A\*，理论增益是支持"非完整约束起点朝向"——但 `map_planner` 已经做了 heading-aware，这部分增量收益有限。Hybrid A\* 真正的价值在窄通道 + 倒车场景，**对 Go2 trot 步态价值有限**，可后置评估。

---

## 5. 后端可替换性矩阵

| 后端 | 输入接口要求 | 输出 | 与 whole.launch 兼容性 |
|---|---|---|---|
| NeuPAN-fork（当前） | `/initial_path` + `/scan_360` + `/odin1/odometry` | `/cmd_vel` + `/neupan/arrive` | ✅ 原生 |
| NeuPAN 原版 (`hanruihua/NeuPAN`) | 同上（话题名都在 config.yaml 可改） | 同上 | ✅ 替换 `NeuPAN/` 子模块即可（README L69-72 直接说明） |
| **AL-iLQR**（方案 D） | 同上 | `/cmd_vel`（`/neupan/arrive` 可仿照发） | ⚠ 需自己写 ROS 节点，订阅 3 个话题，发 1+1 个；本仓库提供了完整模板（NeuPAN python 节点 ~1690 行可作样本） |
| ETH-SRU（方案 A） | depth + odom + goal_pose（**不要 /initial_path**） | `/cmd_vel` | ❌ 不兼容当前栈，必须独立跑 |
| model_planner DWA / move_base | `/map` + `/move_base_simple/goal` | `/cmd_vel` | ⚠ 自带 global planner，与 map_planner 冲突 |

---

## 6. 接入要点（给方案 B / C / D 共用）

### 6.1 方案 C（NeuPAN-fork）= 当前栈，零改动

直接 `roslaunch map_planner whole.launch map_file_name:=<map>` + `rosrun neupan_ros neupan_node`（具体启动入口需到狗上看 `OAYN` 工作区的最终命令；本仓库没看到直接的 NeuPAN launch 文件，可能是 user 用 mamba env + `python -m neupan.ros.neupan_ros` 直接跑）。

### 6.2 方案 B（NeuPAN 原版）

按 README L69-72：
```bash
cd compare/Odin-Nav-Stack-New/NeuPAN
rm -rf *   # 清空 fork
git clone https://github.com/hanruihua/NeuPAN.git .
```
注意原版没有 `manifold` fork 加的 `odom_feedback`、`Smith predictor`、`stuck-escape` 等改进，需要把 `neupan/ros/configs/config.yaml` 里那些段标 disabled（或者直接用原版默认 config）。

待办：拉原版后核对 `config.yaml`/`planner.yaml` 字段差异。

### 6.3 方案 D（AL-iLQR）= 写一个 `ilqr_local_planner` ROS 节点

按 `compare/al-ilqr-pro-analysis.md` §6 的最短路径建议，**借用 Odin-Nav-Stack 的全局前端**：

```python
# 伪代码结构
class ILQRLocalPlanner:
    def __init__(self):
        rospy.Subscriber("/initial_path",    Path,       self.path_cb)
        rospy.Subscriber("/scan_360",        LaserScan,  self.scan_cb)
        rospy.Subscriber("/odin1/odometry",  Odometry,   self.odom_cb)
        self.cmd_pub    = rospy.Publisher("/cmd_vel",     Twist,    queue_size=1)
        self.arrive_pub = rospy.Publisher("/neupan/arrive", Empty,  queue_size=1)
        # 调 al-ilqr-pro 的 ALILQRSolver
```

这样可以**完全复用 map_planner 的 A\* 前端 + fake360 + goal_state_machine 自动 replan**，只换最后一段优化求解器。

### 6.4 必须保留的话题别名

- `/neupan/arrive`：goal_state_machine 监听这个事件做自动 replan。**AL-iLQR 节点必须也发**（每到子目标点发一次空消息），否则 replan 链路断掉。
- `/odin1/odometry` vs `/odin1/odometry_highfreq`：NeuPAN 用 10 Hz 版做 MPC 反馈；ETH-SRU 用 400 Hz 版做"状态估计"。两者用途不同，不能混。

---

## 7. 部署可行性

**已经部署成功**（你本地 `宇树go2运行Odin-Nac-Stack.md` 记录完整流程）。Jetson NX 上跑通的实证 + Docker setup（`docs/DOCKER_SETUP.md`）说明无硬件门槛。

ROS2 开发机问题：你说"开发机是 ros2 的无法直接运行"——这没问题，**开发机只做静态分析 + 写代码**，所有运行/调试都在狗上做。本次同步就是为了在开发机做代码分析。

---

## 8. 待澄清 / 风险清单

- **NeuPAN-fork 的"odin-stack-fixes" 改动列表**：README L41-43 提到 heading-aware A* + dynamic obstacle memory + Smith predictor + stuck-escape。换成原版后，**这些改动会丢失**，性能可能下降。需要：
  1. 跑一遍 fork 版（方案 C）
  2. 跑一遍原版（方案 B）
  3. 对比指标，验证 fork 的增量价值
- **dev_docs/neupan_issues_to_fix.md** 看起来记录了大量历史 bug 和 fix——后续若在方案 C 上做调参，应当先读它。
- **goal_state_machine 与 AL-iLQR 接口**：`goal_state_machine.cpp` 监听 `/neupan/arrive` 自动 replan，话题名硬编码。我们 AL-iLQR 节点要么发 `/neupan/arrive`（粘 NeuPAN 命名空间），要么改 goal_state_machine 的 remap。**前者更省事**。
- **map 文件与场景**：每个场景需要单独 PGM 地图（`map_planner/maps/`）。实验时录 bag 前要先建图。`scripts/map_recording.sh` 流程已就绪。
- **没有 launch 把所有节点一起拉起的全量入口**：当前 `whole.launch` 只到 NeuPAN 之前，NeuPAN 是单独跑的 python 进程。我们若要写"一键启动方案 D"的 launch，要把 ilqr_local_planner 节点也包进去。
- **fake360 cell_decay_sec=3.0** 是经过血泪调过的（README L41-48 历史）。换 AL-iLQR 后这个参数不一定还合适——AL-iLQR 横向避障比 NeuPAN 更激进/保守？需要实测。
- **TF 树验证**：`odin1_base_link → base_link` static TF `-0.3 0 0`（`whole.launch:124`），这是 Odin1 到狗 base_link 的偏移。AL-iLQR 节点的"机器人当前位姿"必须从 TF 里取（不能直接用 odom topic 的 `pose`，因为 odom 是 `odin1_base_link` 帧的）。
