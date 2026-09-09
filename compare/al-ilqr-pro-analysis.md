# AL-iLQR-pro 工程接入分析（方案 D）

> 目标：判断 `compare/al-ilqr-pro` 能否接入 Go2 + Jetson NX 的 Odin-Nav-Stack 局部规划链路，作为四方案对比中的 D 路。
> 范围：仅读分析，不改源码。参考 `详细文档/AL-iLQR详细推导.md`，结合 codegraph 与代码核读。
> 命名空间：`my_al_ilqr`。

---

## 1. 仓库定位

教学性 AL-iLQR 实验台，自洽实现："动力学 + 代价 + 约束 + AL-iLQR 求解器 + ImGui 可视化"。
- **核心库 `my_al_ilqr_core`** 只依赖 Eigen3（`CMakeLists.txt:70-126`）。
- 14 个 `phase*` 可执行示例（`examples/`）+ 1 个 `imgui_frontend` 交互前端（`apps/`）。
- 已实现：静态/动态圆形障碍、矩形障碍（LSE 平滑 / 精确多边形碰撞）、车辆多圆近似、解析导数、AL 外层迭代。
- 论文对齐：AL-iLQR Tutorial + ALTRO。

---

## 2. 模块 I/O 速查

| 模块 | 头文件 | 关键 I/O |
|---|---|---|
| **核心求解器** `ALILQRSolver` | `include/al/al_ilqr_solver.hpp:40-52` | 入：`ConstrainedOptimalControlProblem&`, `ALILQROptions`, `Solve(initial_state, initial_controls)`。出：`ALILQRResult{Trajectory, converged, final_violation, outer_logs}` |
| **内层 iLQR** `ILQRSolver` | `include/ilqr/ilqr_solver.hpp:32-79` | 入：`OptimalControlProblem&`, `ILQROptions(derivative_mode, max_iter, lm reg, line search)`。出：`Trajectory` + 反馈增益/前馈项历史 |
| **OCP** `OptimalControlProblem` | `include/problems/optimal_control_problem.hpp:12-45` | 持有 `DynamicsModel + stage/terminal CostFunction`，提供 `Rollout` 与 `TotalCost` |
| **C-OCP** `ConstrainedOptimalControlProblem` | `include/problems/constrained_optimal_control_problem.hpp` | 基于 OCP 追加 stage / terminal 不等式 + 等式约束（`AddStageConstraint(k, ...)`）|
| **动力学** | `include/dynamics/` | `LinearPointMassModel`、`UnicycleModel`（3 状态）、`KinematicBicycleModel`（4 状态） |
| **代价** | `include/cost/`、`include/autodrive/` | `QuadraticCost`、`LaneTrackingCost`、`GuidanceTrackingCost`、`ControlRateCost(jerk+steer_rate)` |
| **约束** | `include/autodrive/` | `CircularObstacleConstraint`、`MultiCircleVehicleObstacleConstraint`（多圆车体 vs 圆障碍）、`LSEPolygonObstacleConstraint`、`PolygonCollisionConstraint`（精确）、`MultiCircleRoadBoundaryConstraint`、`SpeedLimitConstraint`、`ControlBoxConstraint`、`TerminalGoalConstraint` |
| **前端** | `include/autodrive/` | `GlobalGuidancePlanner`（DP/lattice → topology-aware）、`SLDPPlanner`、`LatticeInitialGuess`、`PurePursuitTracker`、`GuidanceTracker`（用 AL-iLQR 跟踪 guidance） |
| **场景装配** | `include/autodrive/demo_scenario.hpp` | `CreateSingleStaticObstacleTestScenario` / `CreateDynamicObstacleDemoScenario` / `CreateDynamicObstacleSpeedStudyScenario`，输出可直接交给 `ALILQRSolver` 的 `AutodriveDemoScenario` 包 |

---

## 3. 当前前端 & 数据流

`demo_scenario.hpp:20-30` 提供两组开关：

```
InitGuessMode    ∈ {kNone, kGlobalGuidance, kSLDP}    // 初始 guidance 来源
InitControlsMode ∈ {kPlanner, kPP, kPPILQR}            // 初始控制序列生成方式
```

**默认数据流**（静态障碍 demo，`StaticObstacleScenarioConfig`，`demo_scenario.hpp:93-94`）：

```
StraightReferenceLine
   ↓
GlobalGuidancePlanner.Generate(state, obstacles, ...) → GuidanceCandidate[]+selected GuidancePoint[]
   ↓ (init_controls_mode)
   ├ kPlanner : 候选自带的 controls 序列
   ├ kPP      : PurePursuitTracker.Track(guidance, x0, dynamics, ...) → controls (kinematic rollout)
   └ kPPILQR  : 先 PP，再用 GuidanceTracker(=AL-iLQR + tracking-cost) 二次平滑 → controls
   ↓
ALILQRSolver.Solve(x0, initial_controls)   // 内含: AL 外层 + iLQR 内层
   ↓
Trajectory(states, controls)
```

**动态障碍 demo**（`examples/phase11_dynamic_obstacle_demo.cpp:13-18`）走的是简化路径：
直接构造场景骨架 → 用恒定加速度 ramp 作 `initial_controls` → 不经 PP 直接 AL-iLQR。这条路径更接近"线上规划"使用方式。

### 3.1 关于 PP

PP 不是这个仓库的"前端"，而是"为 iLQR 提供初值"的内嵌轨迹跟踪器（`PurePursuitTracker`，`include/autodrive/pure_pursuit_tracker.hpp:31-44`，纯静态方法，不持状态）。真正承担"全局路径生成 / 候选筛选"的是 `GlobalGuidancePlanner` 和 `SLDPPlanner`，它们只支持**直线参考线**（`StraightReferenceLine`），并依赖 `lateral_offset × speed_ratio × num_layers` 的 lattice 枚举。

### 3.2 能否接入我们的前端

可以，且有两种切入位置：

- **位置 A（推荐）：替换整个前端**，直接喂 `initial_controls`。
  AL-iLQR 求解器只要 `(initial_state, std::vector<Vector> initial_controls)`（`al_ilqr_solver.hpp:46`）。我们可用 Odin-Nav-Stack 现有前端（或 Hybrid A\*）输出的路径 → 由我们自行做一次 PP/kinematic rollout 得到 `controls[H]` → 直接调 `Solve`。完全跳过 `GlobalGuidancePlanner` / `SLDPPlanner` / `StraightReferenceLine`。
- **位置 B：用 guidance 接口**，调用 `GuidancePointsToControls(...)`（`global_guidance_planner.hpp:150-154`）把我们的路径折算成 controls。需保证字段含 `x,y,yaw,speed`，并指定 wheelbase。

**前提**：所有 `autodrive` 约束都硬编码 `StateDim()==4` / `ControlDim()==2`（bicycle，状态 `[x,y,yaw,v]`，控制 `[a,δ]`），见 `circular_obstacle_constraint.hpp:12-13`、`multi_circle_vehicle_obstacle_constraint.hpp:16-17`、`lse_polygon_obstacle_constraint.hpp:18-19`。Go2 是差速底盘，**要么**把它视作虚拟自行车（有效 wheelbase 估一个值，最自然），**要么**改用 `UnicycleModel`（3 状态，`include/dynamics/unicycle_model.hpp:9-10`）**并新写一份 3D 约束**。前者工作量小，对 Go2 trot 速度足够。

---

## 4. 动态避障能力判定

**结论：具备时间相关的、prediction-aware 的动态避障，但不做"感知/预测"本身。**

证据链：
1. `PredictedObstacle.centers` 是 `vector<Vector>`（`global_guidance_planner.hpp:19-27`），一个 timestep 一个中心点。
2. `MakeConstantVelocityPrediction(...)`（`global_guidance_planner.hpp:130-137`）默认按恒速向前外推。
3. 场景装配 `AddObstacleConstraints` 按时间步逐个 stage 添加约束（`src/autodrive/demo_scenario.cpp:178-213`）：
   ```
   for (k = 0; k < H; ++k)
       constrained_problem->AddStageConstraint(
           k, MultiCircleVehicleObstacleConstraint(geom, centers[k].x, centers[k].y, R));
   ```
   即对 horizon 内每一帧 k，使用对应时刻的预测障碍位置 (cx[k], cy[k]) 形成不等式约束。
4. `phase11_dynamic_obstacle_demo.cpp:13-18` 与 `phase12` 完整跑这条路径。

也就是说：
- 求解器把"未来 H 帧障碍位置"作为输入条件，规划期间假定 prediction 不变。
- **不做** EKF/Kalman/学习类预测；预测由调用方传入（默认恒速）。
- 若要换运动模型（CV→CTRA/Social-LSTM/…），只需替换 `obstacle_centers` 的生成函数，约束结构保持不变。
- 矩形动态障碍同样支持（`ObstacleType::kPolygonLSE`/`kPolygonExact`，`demo_scenario.cpp:189-211`）。

工程含义：对接 Go2 时，**必须**在上游加一个"动态障碍检测 + 速度估计 + 横向预测"模块（最简：从 Odin 点云聚类得到障碍 + CV 外推），否则只能做静态避障。

---

## 5. Jetson NX 编译可行性

**结论：核心库可直接在 NX 上编译，无 GPU 依赖；UI 前端需关闭或单独配置。**

| 项 | 评估 | 备注 |
|---|---|---|
| C++ 标准 | C++17 | g++ 9（Ubuntu 20.04 自带）OK，`CMakeLists.txt:4` |
| CMake | ≥ 3.16 | Ubuntu 20.04 自带 3.16.3 OK |
| Eigen3 | 3.3+ | `sudo apt install libeigen3-dev`（aarch64 包齐全） |
| 核心库依赖 | 仅 Eigen3 | `CMakeLists.txt:70-126`，无 GPU / 无 OpenMP / 无 BLAS 强依赖 |
| ImGui 前端 | OpenGL + GLFW + ImGui | NX 一般 headless，**建议 `-DMY_AL_ILQR_ENABLE_IMGUI_FRONTEND=OFF`**（`CMakeLists.txt:8-10` 提供开关） |
| FetchContent | GLFW/ImGui 在线下载 | 关 UI 后这步整段跳过；要保留 UI 时 NX 联网或预先 mirror `_deps/glfw-src`、`_deps/imgui-src`（`CMakeLists.txt:22-45` 已支持本地目录覆盖） |
| Python3 + NumPy | 仅 `phase17_guidance_tracker_test` 使用 | `find_package(Python3 ... QUIET)` 找不到就跳过（`CMakeLists.txt:486-518`），不影响主链路 |
| install_deps.sh | apt 包列表对 NX 完全兼容 | 见 `scripts/install_deps.sh:31-49`，关 UI 后只剩 `build-essential cmake libeigen3-dev`，2–3 分钟搞定 |
| 架构 | 纯 C++/Eigen，aarch64 安全 | 无 x86 intrinsics |

**NX 上推荐配置命令**：
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DMY_AL_ILQR_ENABLE_IMGUI_FRONTEND=OFF
cmake --build build -j$(nproc)
```
这样产物只有 `phase0..phase13_*` 示例 + `my_al_ilqr_core.a`，可直接链接进 ROS 节点。

**风险点**：CMake `FETCHCONTENT_UPDATES_DISCONNECTED ON`（`CMakeLists.txt:7`）+ 关 UI 后无 FetchContent 调用，所以离线编译可行。但当前仓库**没有提供 ROS 节点包装**，需要我们自己写一个 nodelet/封装把 `ALILQRSolver` 暴露成 `cmd_vel` 发布者。

---

## 6. 接入 Go2 + Odin-Nav-Stack 的最短路径建议

1. **跳过 `demo_scenario` 装配**，写一个 ROS 节点 `ilqr_local_planner`，直接组装：
   `KinematicBicycleModel`（伪 wheelbase）+ `QuadraticCost`（参考跟踪）+ `MultiCircleVehicleObstacleConstraint`（每个动态障碍每帧）+ `ControlBoxConstraint` + 路边界（可选）+ `ALILQRSolver`。
2. **前端**：第一阶段直接复用 Odin-Nav-Stack 当前前端（输出 `nav_msgs/Path`），按 `(dt, horizon)` 重采样并用 kinematic rollout 转 `initial_controls`。后续按计划替换 Hybrid A\*。
3. **动态障碍预测**：第一阶段静态障碍优先，约束以静态形式入；动态避障留到加入感知预测模块后再开。
4. **状态维度对齐**：Go2 用伪 bicycle 4 维状态最省事；测一下转弯半径是否合理，不行再改 unicycle + 重写约束。
5. **打包**：核心库做成 catkin 子目录或 catkin 外部 CMake，target_link 到 ROS 包；NX 上不带 UI 编译。

---

## 7. 待澄清 / 风险清单

- **Go2 模型选择**：bicycle 伪化 vs unicycle + 重写约束。bicycle 路径短但需调 wheelbase；unicycle 代价是 `autodrive/` 下 6 个约束类要改 `StateDim()`、Jacobian、Evaluate。建议先 bicycle 跑通，跑不动再切。
- **AL-iLQR 求解时间**：仓库未给 NX 上的实测耗时。phase11 默认 `horizon=80, dt=0.1`、`max_outer_iter=18`，数值差分模式下单次解时间在桌面 CPU 约 50–200ms；NX 上需实测。可先用 `derivative_mode = kAnalytical`（`ilqr_solver.hpp:13`）+ 关 polygon 精确约束（`PolygonCollisionConstraint` 仍是数值导数）以保速度。
- **直线参考线限制**：`StraightReferenceLine` 是仓库内 guidance/SLDP/lattice 的基础假设，曲线参考线需要自己提供。如果绕过其前端（位置 A），此限制不影响我们。
- **没有 ROS wrapper**：需要自己写。建议放在 `compare/al-ilqr-pro/ros_wrapper/`（或开发机的新工作区），保持上游仓库干净便于追更。
- **动态障碍预测来源缺失**：仓库假定外部提供 `centers[k]`；Odin-Nav-Stack 现有栈是否有动态障碍模块需确认。第一版可只跑静态避障。
- **测试可用性**：仓库自带 ctest（`CMakeLists.txt:285+`）覆盖 phase0–phase16，可直接作为接入后第一轮回归基准。

---

## 8. 下一步动作（待审核确认后启动）

按 `compare/执行计划.md` 的"后续任务"序列继续：

1. 在狗电脑连接后，先做执行计划 §后续任务 1（rsync ETH-SRU & Odin-Nav-Stack）。
2. 用 codegraph 扫描 `compare/` 下三个仓库，建立完整代码图谱（执行计划 §后续任务 2）。
3. 部署 NeuPAN 原始版（方案 B），与 A 一起作为 NeuPAN 内对比基线。
4. 按本文 §6 路径开始 AL-iLQR ROS 包装，目标：能跑通静态障碍直线避障 demo on Jetson NX。
