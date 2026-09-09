# ETH-SRU (sru_nav_go2_ros1) 工程接入分析（方案 A）

> 目标：把"已经部署在 Go2 上的 SRU 学习策略"作为对比基线 A，理清它的输入输出、避障原理、和我们后续要做的 AL-iLQR/NeuPAN 方案有什么本质差别。
> 同步来源：`unitree@192.168.123.18:/home/unitree/code/odin_sru_nav/`（实机已跑）。
> 部署细节见 `compare/ETH-SRU/DEPLOY_GO2_NX.md`（你手编），本文只做架构 + 接入向分析。

---

## 1. 仓库定位

`sru_nav_go2_ros1` 是 **ETH 系 SRU (Self-Recurrent Unit) 学习导航策略**的 ROS1 Noetic + Go2/Odin1 移植，对应论文/原仓库 `sru-robot-deployment`。**纯端到端 RL 策略，没有传统规划/优化**，整个"导航"是一个 LSTM/SRU policy 网络。

> "It is a near-1:1 port of `.../rl_nav_controller.py` with the following substitutions: `/zed/.../depth_registered` → `/odin1/depth_img_competetion`；`/dlio/odom_node/odom` → `/odin1/odometry_highfreq`；`POLICY_SCALE` 从 wheeled B2W 的 `[1.5,1.0,1.0]` 改 Go2 的 `[0.6,0.3,0.6]`" — `src/sru_nav_go2_ros1/README.md:14-25`

`unitree_control` 是配套的 `cmd_vel → SportClient.Move(...)` 桥（`src/unitree_control/src/unitree_vel_controller.cpp:77-80`），AI Walk 模式 + `FreeAvoid(false)` 关闭官方避障（让策略自己负责避障）。

---

## 2. 模块 I/O 速查

| 模块 | 文件 | 关键 I/O |
|---|---|---|
| **策略核心** `LearningModel` | `src/sru_nav_go2_ros1/src/sru_nav_go2/model.py:21-72` | 入：depth `(H,W)` + state(16维: lin_vel + ang_vel + projected_gravity + last_action + target_pos_log4) + LSTM h/c。出：`cmd_vel(3) = tanh(raw_action) * POLICY_SCALE`、`(h_out, c_out)` |
| **VAE encoder** | 同上 `depth_preprocess` (`model.py:82-90`) | depth → resize 40×64 → 2560-d latent |
| **ROS 节点** `NavigationPolicyNode` | `src/sru_nav_go2_ros1/src/sru_nav_go2/navigation_policy_node.py:39-757` | 订阅 depth/odom/joy/goal_pose；发布 `/cmd_vel` + 一堆 rviz Marker |
| **Waypoint 管理** | `src/sru_nav_go2_ros1/src/sru_nav_go2/waypoint_manager.py` | 维护 waypoint 队列，到点切下一个 goal |
| **常量配置** | `src/sru_nav_go2_ros1/src/sru_nav_go2/constants.py` | `POLICY_SCALE=[0.6,0.3,0.6]`、`LOW_PASS_FILTER_COEF=[0.9,0.5,0.5]`、`JOYSTICK_TIMEOUT=15s`、控制频率 5 Hz |
| **YAML 运行参** | `src/sru_nav_go2_ros1/config/sru_nav.yaml` | depth/odom/goal/cmd_vel topic、`min_depth=0.25`、`max_depth=10.0`、`control_frequency=5.0`、`policy_scale`、`pose_source(odom\|tf)`、`use_sim`、`require_joystick` |
| **底盘桥** `unitree_vel_controller` | `src/unitree_control/src/unitree_vel_controller.cpp` | 订 `/cmd_vel`；`StandUp → BalanceStand → FreeWalk + FreeAvoid(false)`；`SportClient.Move(vx, vy, ωz)` |

ROS 话题接口：

```
订阅                                         发布
  /odin1/depth_img_competetion (Image 32FC1) → /cmd_vel (Twist) → unitree_control → Go2
  /odin1/odometry_highfreq     (Odometry, ~400Hz)
  /joy                          (Joy, deadman + speed scale)
  /goal_pose                    (PoseStamped, frame=odom)
```

---

## 3. 当前前端 & 数据流

**这是一个"无前端"系统**——策略本身就是端到端"感知 → 动作"。

```
depth(H,W) ─┐
            ├──> NavigationPolicyNode (5 Hz)
odom ───────┤   │
goal_pose ──┘   │ 1) 算 target_vec_b (goal in body frame)
                │ 2) 算 projected_gravity, last_action
joy_throttle ── │ 3) LearningModel.predict():
   (deadman)    │     - VAE(depth) → 2560-d latent
                │     - concat 状态 + latent → LSTM(h,c)
                │     - tanh(raw) * POLICY_SCALE → cmd_vel
                │ 4) 低通滤波: alpha*new + (1-alpha)*prev
                ▼
              /cmd_vel ─> unitree_control ─> Go2 SportClient.Move
```

**关键事实**：
- "全局路径"= **唯一一个 goal 点** + waypoint 队列。没有 A*、没有 costmap，没有任何拓扑/几何规划。
- "局部规划"= **LSTM 隐状态 + 当前深度图**。隐状态相当于策略"记忆"，包括对动态物体的隐含跟踪能力（训练数据决定）。
- 输出是 5 Hz 的 `cmd_vel`，相当于直接生成控制指令，不经过任何 MPC/iLQR/优化。

### 3.1 是否能接入我们的前端？

技术上**可以接**，但接了等于"在端到端策略外面套规划器"，破坏其设计假设：
- 策略已经吃 `target_vec_b`（goal 在 base_link 系的相对方向），把 A* 的下一段 sub-goal 当作 `target_pos_w` 喂给它也能跑——但策略对"近距离 sub-goal"的反应是训练时未必见过的分布。
- 真正合理的"加前端"做法：用 A* / Hybrid A* 生成 waypoint 序列，由 `WaypointManager` 一段一段切换 goal 给策略追。这是当前 README 已经支持的 "waypoint_runner" 模式（`scripts/waypoint_runner.py`）。

### 3.2 是否能拆出"前端只用 Odin-Nav-Stack，后端只用 SRU"？

**不能直接拆**。SRU 策略要的不是路径而是 goal 点。如果硬要把 `/initial_path` 接成策略输入，得：
1. 路径首点附近选一个滚动 lookahead 点作 `target_pos_w`；
2. 重新 reset LSTM 隐状态（每次切点都会 reset）；
3. 损失原始策略的轨迹连续性收益。

实际上 ETH-SRU 不应与 Odin-Nav-Stack 共栈跑，应作为**独立基线**：单独一个 launch（`launch_sru_nav.sh`），单独的 `/cmd_vel`。

---

## 4. 避障能力判定

**结论：纯学习避障，无显式建模；动态避障靠 LSTM 时序记忆 + depth 即时反应。**

证据链：
1. 策略输入唯一感知通道是 **depth image**（VAE 编码为 2560-d latent），没有点云、没有 LaserScan、没有障碍物预测话题。
2. `unitree_control` 强制 `FreeAvoid(false)`（`unitree_vel_controller.cpp:49`），关闭 Go2 内置避障——意味着避障**完全交给策略本身**。
3. 没有任何 collision constraint、costmap、SDF——避障只能体现为"策略在训练时见到障碍 → 学到打 yaw / 减速"。
4. LSTM 隐状态从 `BUTTON_RESET_HIDDEN_STATE` reset 之后才清，故策略对**近期看到过又转出 FOV 的障碍有几秒的隐式记忆**——但这是黑箱、不可调、不可解释。

工程含义：
- 速度优势：5 Hz 推理 + ONNX，**无优化求解**，NX 上 CPU EP 都能跑（DEPLOY_GO2_NX.md §3.3）。
- 安全短板：
  1. 没有 collision-free 保证；
  2. 动态障碍预测是隐式的，无法主动塞预测进去；
  3. Sim-to-real gap：策略训练在 wheeled B2W，迁移到 Go2 是 zero-shot（README 显式标注未验证）。

---

## 5. 部署可行性

**已经部署成功**（你的 `DEPLOY_GO2_NX.md` 已记录）。这里只补三条本次分析中发现的**约束**：

| 约束 | 来源 | 含义 |
|---|---|---|
| Python 3.8 强约束 | `setup_conda_env.sh` + `cv_bridge` 需要 ROS Noetic 系统 python ABI | conda env 必须 py3.8，否则 `cv_bridge` import 失败 |
| `onnxruntime-gpu` 需要 JetPack wheel | `DEPLOY_GO2_NX.md:41-46` | 装错版本（pip 默认 x86）直接报 illegal instruction |
| 必须有手柄或 `--no-deadman` | `constants.py:16`、`navigation_policy_node.py:347-356` | `JOYSTICK_TIMEOUT=15s`，超时强制 cmd_vel=0 |
| Odin1 dense depth 在 NX 上是 CPU 计算瓶颈 | `README.md:121-124` | 必须先确认 `rostopic hz /odin1/depth_img_competetion` ≈ 10 Hz 才能跑 |
| odom twist 坐标系**必须实测** world/body | `DEPLOY_GO2_NX.md` §5 | 弄反 → cmd_vel 抖动剧烈 |

---

## 6. 在对比实验中的定位

- **基线 A**：作为"端到端 RL"的代表，对比"传统规划+优化"的方案 C/D（NeuPAN / AL-iLQR）。
- **不接 Odin-Nav-Stack 全栈**：独立跑 `launch_sru_nav.sh` + `unitree_control`，goal 通过 `rostopic pub /goal_pose ...` 或 `waypoint_runner.py` 给。
- **录包对象**：`/odin1/odometry_highfreq` (轨迹) + `/cmd_vel` + `/joy` + `/goal_pose` + `/odin1/depth_img_competetion`（可选，文件大）。
- **统一评估指标**（按 `执行计划.md`）：SR、SPL、轨迹长度、控制能耗、最小障碍距离、避障次数、震荡、卡死。

---

## 7. 待澄清 / 风险清单

- **唯一可调"避障性能"**：只有 `POLICY_SCALE` 和 `LOW_PASS_FILTER_COEF`。无法像 NeuPAN/iLQR 那样调权重让它"更保守"或"更激进"——动作分布是训练好的网络决定的。
- **缺乏复现性**：不同 seed 训练的策略行为差异巨大；当前部署用的是仓库附带 onnx，不能保证与论文同一权重。
- **goal frame 必须严格对齐**：`/goal_pose.header.frame_id` 必须等于 `/odin1/odometry_highfreq.header.frame_id`（默认 `odom`），否则被忽略（`DEPLOY_GO2_NX.md` §6 已记）。
- **与 Odin-Nav-Stack 共存**：两个栈都会发 `/cmd_vel` + 都依赖 `unitree_control`。**同一时刻只能跑一个**，不能同时打开 `sru_nav_go2.launch` 和 Odin 的 `whole.launch`。这点在实验设计里要写清楚。
- **狗的稳定性**：策略输出 vy（横向速度），Go2 横向运动能力弱，`LATERAL_VELOCITY_SCALE=0.6` 是额外阻尼。若狗摔倒/抖动，先把 vy scale 调到 0.3 看看。
- **没有完整 ROS bag 评估脚本**：当前仓库给的是 `waypoint_runner.py` 跑离线路径，但没有"按指标算 SR/SPL"的脚本——这部分要自己写（执行计划 §后续任务 8）。
