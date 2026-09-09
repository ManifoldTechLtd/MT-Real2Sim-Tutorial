# 最终篇：项目全览——构建、运行与完整调用链

> **目标**：从"拿到代码"到"跑通求解"的完整指引，以及从 `main()` 入口到每一行核心计算的函数调用链路图

---

## F.1 项目目录结构

```
MY-AL-iLQR/
│
├── CMakeLists.txt                 ← 构建配置（所有目标定义在此）
├── main.cpp                       ← matplotlib GUI 入口（planner_gui）
│
├── include/                       ← 头文件（按模块划分子目录）
│   ├── core/
│   │   ├── types.hpp              ← Vector, Matrix 类型别名
│   │   └── trajectory.hpp         ← Trajectory 轨迹容器
│   │
│   ├── dynamics/
│   │   ├── dynamics_model.hpp     ← 动力学抽象接口
│   │   ├── linear_point_mass_model.hpp
│   │   ├── unicycle_model.hpp
│   │   └── kinematic_bicycle_model.hpp
│   │
│   ├── cost/
│   │   ├── cost_function.hpp      ← 代价函数抽象接口
│   │   └── quadratic_cost.hpp
│   │
│   ├── constraints/
│   │   ├── constraint_function.hpp ← 约束抽象接口
│   │   ├── control_box_constraint.hpp
│   │   └── terminal_goal_constraint.hpp
│   │
│   ├── problems/
│   │   ├── optimal_control_problem.hpp      ← 无约束 OCP
│   │   └── constrained_optimal_control_problem.hpp ← 带约束 OCP
│   │
│   ├── lqr/
│   │   └── finite_horizon_lqr_solver.hpp    ← LQR 求解器
│   │
│   ├── ilqr/
│   │   └── ilqr_solver.hpp                  ← iLQR 求解器
│   │
│   ├── al/
│   │   ├── augmented_lagrangian_cost.hpp    ← AL 代价转换
│   │   └── al_ilqr_solver.hpp              ← AL-iLQR 求解器
│   │
│   ├── autodrive/
│   │   ├── straight_reference_line.hpp
│   │   ├── lane_tracking_cost.hpp
│   │   ├── vehicle_bicycle_config.hpp
│   │   ├── vehicle_circle_approximation.hpp
│   │   ├── road_boundary_constraint.hpp
│   │   ├── speed_limit_constraint.hpp
│   │   ├── circular_obstacle_constraint.hpp
│   │   ├── multi_circle_vehicle_obstacle_constraint.hpp
│   │   └── demo_scenario.hpp               ← 场景工厂
│   │
│   └── visualization/
│       ├── trajectory_writer.hpp
│       ├── obstacle_prediction_writer.hpp
│       └── al_ilqr_log_writer.hpp
│
├── src/                           ← 与 include/ 一一对应的实现文件
│   ├── core/trajectory.cpp
│   ├── dynamics/{linear_point_mass_model, unicycle_model, kinematic_bicycle_model}.cpp
│   ├── cost/quadratic_cost.cpp
│   ├── constraints/{constraint_function, control_box_constraint, terminal_goal_constraint}.cpp
│   ├── problems/{optimal_control_problem, constrained_optimal_control_problem}.cpp
│   ├── lqr/finite_horizon_lqr_solver.cpp
│   ├── ilqr/ilqr_solver.cpp
│   ├── al/{augmented_lagrangian_cost, al_ilqr_solver}.cpp
│   ├── autodrive/{demo_scenario, lane_tracking_cost, road_boundary_constraint, ...}.cpp
│   └── visualization/{trajectory_writer, obstacle_prediction_writer, al_ilqr_log_writer}.cpp
│
├── examples/                      ← 分阶段示例程序
│   ├── phase0_linear_rollout.cpp
│   ├── phase1_unicycle_rollout.cpp
│   ├── phase1_bicycle_rollout.cpp
│   ├── phase2_linear_lqr_demo.cpp
│   ├── phase3_unicycle_ilqr_demo.cpp
│   ├── phase4_unicycle_ilqr_stable_demo.cpp
│   ├── phase5_constraint_evaluation_demo.cpp
│   ├── phase6_augmented_lagrangian_demo.cpp
│   ├── phase7_al_ilqr_demo.cpp
│   ├── phase8_autodrive_demo.cpp          ← 静态障碍物避障（核心 demo）
│   ├── phase9_autodrive_visualization_demo.cpp
│   ├── phase10_numerical_enhancement_demo.cpp
│   ├── phase11_dynamic_obstacle_demo.cpp  ← 动态障碍物避障
│   ├── phase12_dynamic_obstacle_speed_sweep_demo.cpp
│   └── phase13_single_static_obstacle_demo.cpp
│
├── tests/                         ← 单元测试
│   ├── phase0_smoke_test.cpp
│   ├── phase1_{unicycle,bicycle}_test.cpp
│   ├── phase2_lqr_test.cpp
│   ├── phase3_ilqr_test.cpp
│   ├── ...
│   └── phase13_vehicle_geometry_test.cpp
│
├── apps/
│   └── rerun_imgui_frontend.cpp   ← Dear ImGui 可视化前端（imgui_frontend）
│
├── config/                        ← INI 配置文件（车辆参数等，如 vehicle_bicycle_config.ini）
├── paper/                         ← 参考论文 PDF
├── ref/                           ← 参考实现代码（仅参考，非本项目代码）
└── guide/                         ← 本开发指南
```

---

## F.2 构建与运行

### F.2.1 依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| CMake | ≥ 3.16 | 构建系统 |
| Eigen3 | ≥ 3.3 | 线性代数库 |
| Python3 | + NumPy | matplotlib 可视化（planner_gui 需要） |
| OpenGL + GLFW | -- | Dear ImGui 前端（可选，imgui_frontend 需要） |

### F.2.2 编译

```bash
# 在项目根目录
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### F.2.3 运行示例

```bash
# 分阶段示例 (从简单到复杂)
./phase0_linear_rollout          # 线性点质量模型 rollout
./phase1_bicycle_rollout         # 自行车模型 rollout
./phase2_linear_lqr_demo         # LQR 求解
./phase3_unicycle_ilqr_demo      # 无约束 iLQR
./phase7_al_ilqr_demo            # 带约束 AL-iLQR
./phase8_autodrive_demo          # 自动驾驶静态避障 (核心 demo)
./phase11_dynamic_obstacle_demo  # 动态障碍物避障

# GUI 前端
./planner_gui                    # matplotlib 交互式界面
./imgui_frontend                 # Dear ImGui 交互式界面 (需要 OpenGL)
```

### F.2.4 运行测试

```bash
cd build
ctest --output-on-failure
```

测试列表：

| 测试 | 验证内容 |
|------|---------|
| `phase0_smoke_test` | 基础设施正确性 |
| `phase1_unicycle_test` / `phase1_bicycle_test` | 动力学模型 |
| `phase2_lqr_test` | LQR 求解收敛性 |
| `phase3_ilqr_test` | iLQR 求解收敛性 |
| `phase4_ilqr_stability_test` | iLQR 正则化稳定性 |
| `phase5_constraints_test` | 约束评估正确性 |
| `phase6_al_cost_test` | AL 代价转换正确性 |
| `phase7_al_ilqr_test` | AL-iLQR 完整求解 |
| `phase8_autodrive_test` | 自动驾驶场景求解 |
| `phase9_visualization_export_test` | CSV 导出 |
| `phase10_numerical_enhancement_test` | 数值增强验证 |
| `phase11_dynamic_obstacle_test` | 动态障碍物 |
| `phase12_dynamic_obstacle_speed_sweep_test` | 速度扫描 |
| `phase13_vehicle_geometry_test` | 车体多圆几何 |

---

## F.3 可执行目标总览

```
CMakeLists.txt 中定义的所有目标:

  库:
  ├── my_al_ilqr_core           ← 核心静态库 (所有 src/*.cpp)
  └── imgui                     ← Dear ImGui 静态库 (可选)

  可执行程序:
  ├── planner_gui               ← matplotlib GUI (main.cpp)
  ├── imgui_frontend            ← Dear ImGui GUI (apps/rerun_imgui_frontend.cpp)
  │
  ├── phase0_linear_rollout     ← 示例: 线性 rollout
  ├── phase1_unicycle_rollout   ← 示例: 独轮车 rollout
  ├── phase1_bicycle_rollout    ← 示例: 自行车 rollout
  ├── phase2_linear_lqr_demo    ← 示例: LQR
  ├── phase3_unicycle_ilqr_demo ← 示例: 无约束 iLQR (使用 ILQROptions 默认正则化)
  ├── phase4_unicycle_ilqr_stable_demo ← 示例: 无约束 iLQR (显式调参的正则化)
  ├── phase5_constraint_evaluation_demo ← 示例: 约束评估
  ├── phase6_augmented_lagrangian_demo  ← 示例: AL 代价
  ├── phase7_al_ilqr_demo       ← 示例: AL-iLQR
  ├── phase8_autodrive_demo     ← 示例: 自动驾驶避障 ★
  ├── phase9_autodrive_visualization_demo ← 示例: 可视化导出
  ├── phase10_numerical_enhancement_demo ← 示例: 数值增强
  ├── phase11_dynamic_obstacle_demo     ← 示例: 动态障碍物 ★
  ├── phase12_dynamic_obstacle_speed_sweep_demo ← 示例: 速度扫描
  └── phase13_single_static_obstacle_demo ← 示例: 单静态障碍物
```

---

## F.4 核心 Demo 代码走读（phase8_autodrive_demo）

以最典型的 `phase8_autodrive_demo.cpp` 为例，走读从 `main()` 到求解完成的每一步：

```cpp
int main() {
  // ① 创建场景 (包含动力学、代价、约束、初始状态、求解器配置)
  const auto scenario = my_al_ilqr::CreateAutodriveDemoScenario();

  // ② 创建求解器
  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);

  // ③ 求解!
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  // ④ 导出结果
  my_al_ilqr::WriteTrajectoryCsv("build/phase8_autodrive_trajectory.csv", result.trajectory);

  // ⑤ 打印日志
  for (const auto& log : result.outer_logs) { ... }
  return 0;
}
```

每一步的详细调用链见后续的完整函数调用链路图。

---

## F.5 完整函数调用链路图

### F.5.1 场景创建阶段

```
main()
  │
  ▼
CreateAutodriveDemoScenario()                     ← demo_scenario.cpp
  │
  ├─→ CreateSingleStaticObstacleTestScenario()
  │     │
  │     ├─→ LoadVehicleBicycleConfig(path)        ← 从 INI 文件加载车辆参数
  │     │     → VehicleBicycleConfig {model, body}
  │     │
  │     ├─→ CreateScenarioSkeleton(config, vehicle_config)
  │     │     │
  │     │     ├─→ VehicleCircleApproximation(body_config)
  │     │     │     → 生成 N 个 VehicleCircle {center_x_body, radius}
  │     │     │
  │     │     ├─→ StraightReferenceLine(0, 0, 0)
  │     │     │
  │     │     ├─→ KinematicBicycleModel(wheelbase)
  │     │     │
  │     │     ├─→ LaneTrackingCost(ref_line, weights...)
  │     │     │
  │     │     ├─→ OptimalControlProblem(dynamics, cost, horizon=55, dt=0.1)
  │     │     │
  │     │     ├─→ ConstrainedOptimalControlProblem(base_problem)
  │     │     │
  │     │     └─→ for k = 0..54:  (逐步添加约束)
  │     │           ├─→ AddStageConstraint(k, ControlBoxConstraint)
  │     │           ├─→ AddStageConstraint(k, RoadBoundaryConstraint)
  │     │           └─→ AddStageConstraint(k, SpeedLimitConstraint)
  │     │
  │     ├─→ obstacle_centers = 填充静态障碍物坐标 (horizon+1 = 56 个相同坐标)
  │     │
  │     ├─→ AddObstacleConstraints(&scenario)
  │     │     └─→ for k = 0..54:
  │     │           └─→ AddStageConstraint(k, MultiCircleVehicleObstacleConstraint)
  │     │
  │     └─→ initial_controls = 启发式初始控制猜测
  │
  └─→ 返回 AutodriveDemoScenario
       {constrained_problem, initial_state, initial_controls, solver_options, ...}
```

### F.5.2 求解阶段（核心!）

```
ALILQRSolver::Solve(initial_state, initial_controls)         ← al_ilqr_solver.cpp
  │
  ├─→ AugmentedLagrangianProblem(constrained_problem, μ₀, φ)  ← 创建 AL 问题
  │     │
  │     ├─→ for k = 0..54:
  │     │     └─→ AugmentedLagrangianKnotCost(base_cost_k, constraints_k)
  │     │           各约束初始化: λ=0, μ=μ₀
  │     │
  │     ├─→ AugmentedLagrangianKnotCost(terminal_cost, terminal_constraints)
  │     │
  │     └─→ OptimalControlProblem(dynamics, AL_stage_costs, AL_terminal_cost, dt)
  │           → 这就是无约束子问题 subproblem_
  │
  ├─→ problem_.BaseProblem().Rollout(x₀, U)                   ← 初始 rollout
  │
  │
  │  ┌══════════════════ 外层循环 (outer_iter = 0..17) ══════════════════┐
  │  ║                                                                   ║
  │  ║  ┌─→ ILQRSolver(*subproblem_, inner_options)                     ║
  │  ║  │     └─→ EnsureWorkspaceSizes()                                ║
  │  ║  │           → 分配 A_[], B_[], expansions_[], K_[], d_[]         ║
  │  ║  │                                                               ║
  │  ║  │                                                               ║
  │  ║  ├─→ ILQRSolver::Solve(x₀, controls)                ← ★ 内层求解 ║
  │  ║  │     │                                                         ║
  │  ║  │     ├─→ problem_.Rollout(x₀, controls)                        ║
  │  ║  │     │     └─→ for k=0..54:                                    ║
  │  ║  │     │           x_{k+1} = dynamics.NextState(x_k, u_k, dt)    ║
  │  ║  │     │             └─→ KinematicBicycleModel::NextState()      ║
  │  ║  │     │                                                         ║
  │  ║  │     ├─→ EvaluateTrajectoryCost(trajectory)                    ║
  │  ║  │     │     └─→ for k: StageCost() + TerminalCost()             ║
  │  ║  │     │           └─→ AugmentedLagrangianKnotCost::StageCost()  ║
  │  ║  │     │                 ├─→ base_cost.StageCost()               ║
  │  ║  │     │                 │     └─→ LaneTrackingCost::StageCost() ║
  │  ║  │     │                 └─→ ConstraintAugmentedTerm() × 每个约束 ║
  │  ║  │     │                       ├─→ constraint.Evaluate(x, u)     ║
  │  ║  │     │                       └─→ 计算 AL 罚项                   ║
  │  ║  │     │                                                         ║
  │  ║  │     │  ┌──── 内层迭代 (iter = 0..34) ────────────────────┐    ║
  │  ║  │     │  │                                                 │    ║
  │  ║  │     │  │  UpdateExpansions(trajectory)                   │    ║
  │  ║  │     │  │    └─→ for k = 0..54:                           │    ║
  │  ║  │     │  │          ├─→ A_k = DynamicsJacobianState()      │    ║
  │  ║  │     │  │          │     └─→ 中心差分: f(x+ε)-f(x-ε)/2ε  │    ║
  │  ║  │     │  │          ├─→ B_k = DynamicsJacobianControl()    │    ║
  │  ║  │     │  │          │     └─→ 中心差分                      │    ║
  │  ║  │     │  │          └─→ CostExpansion = StageCostExpansion()│    ║
  │  ║  │     │  │                ├─→ lx = 中心差分代价对x梯度      │    ║
  │  ║  │     │  │                ├─→ lu = 中心差分代价对u梯度      │    ║
  │  ║  │     │  │                ├─→ lxx = 中心差分代价对x Hessian │    ║
  │  ║  │     │  │                ├─→ lux = 中心差分交叉 Hessian    │    ║
  │  ║  │     │  │                └─→ luu = 中心差分代价对u Hessian │    ║
  │  ║  │     │  │                                                 │    ║
  │  ║  │     │  │  BackwardPass(trajectory, ρ)                    │    ║
  │  ║  │     │  │    ├─→ Vx, Vxx = TerminalCostExpansion(x_N)    │    ║
  │  ║  │     │  │    └─→ for k = N-1 down to 0:                  │    ║
  │  ║  │     │  │          ├─→ Qx = lx + Aᵀ Vx                  │    ║
  │  ║  │     │  │          ├─→ Qu = lu + Bᵀ Vx                  │    ║
  │  ║  │     │  │          ├─→ Qxx = lxx + Aᵀ Vxx A              │    ║
  │  ║  │     │  │          ├─→ Qux = lux + Bᵀ Vxx A              │    ║
  │  ║  │     │  │          ├─→ Quu = luu + Bᵀ Vxx B              │    ║
  │  ║  │     │  │          ├─→ Quu_reg = Quu + ρI                │    ║
  │  ║  │     │  │          ├─→ LLT(Quu_reg)  (Cholesky 分解)     │    ║
  │  ║  │     │  │          │     失败? → return false → ρ增大重试  │    ║
  │  ║  │     │  │          ├─→ K_k = -Quu_reg⁻¹ Qux              │    ║
  │  ║  │     │  │          ├─→ d_k = -Quu_reg⁻¹ Qu               │    ║
  │  ║  │     │  │          ├─→ 累计 ΔV₁, ΔV₂                     │    ║
  │  ║  │     │  │          ├─→ Vx = Qx + Kᵀ Quu d + ...         │    ║
  │  ║  │     │  │          ├─→ Vxx = Qxx + Kᵀ Quu K + ...       │    ║
  │  ║  │     │  │          └─→ Vxx = 0.5(Vxx + Vxxᵀ) 对称化      │    ║
  │  ║  │     │  │                                                 │    ║
  │  ║  │     │  │  ForwardPass(x₀, traj, cost, ΔV₁, ΔV₂)        │    ║
  │  ║  │     │  │    └─→ for α = 1.0, 0.5, 0.25, ...:            │    ║
  │  ║  │     │  │          ├─→ for k = 0..54:                     │    ║
  │  ║  │     │  │          │     δx = x̄_k - x_k                  │    ║
  │  ║  │     │  │          │     ū_k = u_k + α·d_k + K_k·δx      │    ║
  │  ║  │     │  │          │     x̄_{k+1} = f(x̄_k, ū_k, dt)      │    ║
  │  ║  │     │  │          ├─→ cost_new = TotalCost(candidate)    │    ║
  │  ║  │     │  │          ├─→ ratio = (cost-cost_new)/expected   │    ║
  │  ║  │     │  │          └─→ 接受? → 更新轨迹, break            │    ║
  │  ║  │     │  │               拒绝? → α *= 0.5, 继续            │    ║
  │  ║  │     │  │                                                 │    ║
  │  ║  │     │  │  收敛检查: |J_old - J_new| < tolerance?         │    ║
  │  ║  │     │  │    → 是 → break 内层循环                        │    ║
  │  ║  │     │  └──────────────────────────────────────────────────┘    ║
  │  ║  │     │                                                         ║
  │  ║  │     └─→ return trajectory (iLQR 结果)                         ║
  │  ║  │                                                               ║
  │  ║  │                                                               ║
  │  ║  ├─→ problem_.MaxViolation(trajectory)                           ║
  │  ║  │     └─→ EvaluateTrajectory() → 评估所有约束的违约度            ║
  │  ║  │                                                               ║
  │  ║  ├─→ violation < tolerance?                                      ║
  │  ║  │     → 是 → converged = true, break 外层循环                   ║
  │  ║  │                                                               ║
  │  ║  ├─→ al_problem.UpdateDuals(trajectory)                          ║
  │  ║  │     └─→ for k = 0..54:                                        ║
  │  ║  │           stage_costs_[k]->UpdateDuals(x_k, u_k)              ║
  │  ║  │             └─→ for each constraint_data:                      ║
  │  ║  │                   values = constraint.Evaluate(x, u)           ║
  │  ║  │                   等式: λ += μ ⊙ c                             ║
  │  ║  │                   不等式: λ = max(0, λ + μ ⊙ c)                ║
  │  ║  │     terminal_cost_->UpdateDuals(x_N, empty)                    ║
  │  ║  │                                                               ║
  │  ║  ├─→ 进展不足?                                                   ║
  │  ║  │     → al_problem.UpdatePenalties()                            ║
  │  ║  │         └─→ for each cost: μ *= φ                              ║
  │  ║  │                                                               ║
  │  ║  └─→ 回到外层循环顶部                                            ║
  │  ║                                                                   ║
  │  └═══════════════════════════════════════════════════════════════════┘
  │
  └─→ return ALILQRResult {trajectory, converged, violation, logs}
```

---

## F.6 类继承与组合关系图

> **美观版**：在浏览器中打开 [class_diagram.html](class_diagram.html) 可以查看带颜色和交互的完整类图。

### 继承关系

```
DynamicsModel (抽象)                CostFunction (抽象)              ConstraintFunction (抽象)
├── LinearPointMassModel            ├── QuadraticCost                ├── ControlBoxConstraint
├── UnicycleModel                   ├── LaneTrackingCost         ★   ├── TerminalGoalConstraint
└── KinematicBicycleModel   ★       └── ALKnotCost (AL包装)          ├── RoadBoundaryConstraint
                                                                     ├── SpeedLimitConstraint
                                                                     ├── CircularObstacleConstraint
                                                                     └── MultiCircleVehicle...  ★
                                                                 ★ = 自动驾驶场景核心
```

### 组合关系（求解管线）

```
DynamicsModel + CostFunction + horizon + dt
         │
         ▼
OptimalControlProblem                    ← 无约束 OCP
         │ + ConstraintFunction (按 knot point 添加)
         ▼
ConstrainedOptimalControlProblem         ← 带约束 OCP
         │ + λ₀, μ₀, φ
         ▼
AugmentedLagrangianProblem               ← 约束→AL 代价
         │
         │  UnconstrainedSubproblem()
         │  ┌──────────────────────────────────────────┐
         │  │ AL 外层 (ALILQRSolver)                    │
         │  │   固定 λ,μ → iLQR 内层 (ILQRSolver)      │
         │  │     UpdateExpansions → BackwardPass       │
         │  │     → ForwardPass → 新轨迹                │
         │  │   检查收敛 → UpdateDuals → UpdatePenalties │
         │  └──────────────────────────────────────────┘
         ▼
ALILQRResult {trajectory, converged, max_violation, outer_log}
```

---

## F.7 数据流全景图

```
用户输入                        内部处理                      输出
─────────                      ────────                     ──────

VehicleBicycleConfig ─┐
(车辆参数: 轴距,       │
 车身尺寸, 控制极限)    │
                      │
ScenarioConfig ───────┼──→ CreateDemoScenario()
(障碍物位置,速度,       │         │
 道路宽度, 代价权重)    │         ├──→ KinematicBicycleModel
                      │         ├──→ LaneTrackingCost
initial_state ────────┤         ├──→ VehicleCircleApproximation
  [x₀,y₀,θ₀,v₀]      │         ├──→ 各种 Constraint
                      │         └──→ ConstrainedOptimalControlProblem
initial_controls ─────┘               │
  [u₀,...,u₅₄]                       │
                                      ▼
                              AugmentedLagrangianProblem
                                      │
                    ┌─────────────────┼──────────────────────┐
                    │                 ▼                       │
                    │         OptimalControlProblem           │
                    │         (无约束, 代价含 AL 项)           │
                    │                 │                       │
                    │                 ▼                       │
                    │           ILQRSolver.Solve()            │
                    │           ┌────────────────┐            │
                    │           │ UpdateExpansions│            │
                    │   内层    │   A_k, B_k     │            │
                    │   循环    │   lx,lu,lxx... │            │
                    │           ├────────────────┤            │
                    │           │ BackwardPass   │            │
                    │           │   K_k, d_k     │            │
                    │           ├────────────────┤            │
                    │           │ ForwardPass    │            │
                    │           │   新轨迹        │            │
                    │           └───────┬────────┘            │
                    │                   │                     │
                    │   外层            ▼                     │
                    │   循环     检查约束收敛                   │
                    │            │                            │
                    │            ├─→ UpdateDuals (λ更新)       │
                    │            └─→ UpdatePenalties (μ更新)   │
                    │                                        │
                    └────────────────────────────────────────┘
                                      │
                                      ▼
                              ALILQRResult
                              ├── trajectory  ──→ WriteTrajectoryCsv()  ──→ CSV
                              ├── converged                                文件
                              ├── violation
                              └── outer_logs  ──→ WriteALILQROuterLogCsv()
```

---

## F.8 关键函数签名速查表

### 核心求解器

| 函数 | 文件 | 作用 |
|------|------|------|
| `ALILQRSolver::Solve(x₀, U₀)` | `al_ilqr_solver.cpp` | AL 外层循环入口 |
| `ILQRSolver::Solve(x₀, U₀)` | `ilqr_solver.cpp` | iLQR 内层循环入口 |
| `ILQRSolver::UpdateExpansions(traj)` | `ilqr_solver.cpp` | 计算 A,B 和代价展开 |
| `ILQRSolver::BackwardPass(traj, ρ, ...)` | `ilqr_solver.cpp` | Riccati 递推求 K,d |
| `ILQRSolver::ForwardPass(x₀, traj, ...)` | `ilqr_solver.cpp` | 线搜索闭环 rollout |
| `FiniteHorizonLQRSolver::Solve()` | `finite_horizon_lqr_solver.cpp` | LQR Riccati 递推 |

### 问题建模

| 函数 | 文件 | 作用 |
|------|------|------|
| `OptimalControlProblem::Rollout(x₀, U)` | `optimal_control_problem.cpp` | 前向仿真 |
| `OptimalControlProblem::TotalCost(traj)` | `optimal_control_problem.cpp` | 计算轨迹总代价 |
| `ConstrainedOptimalControlProblem::AddStageConstraint(k, c)` | `constrained_optimal_control_problem.cpp` | 添加阶段约束 |
| `ConstrainedOptimalControlProblem::MaxViolation(traj)` | `constrained_optimal_control_problem.cpp` | 最大违约度 |

### 增广拉格朗日

| 函数 | 文件 | 作用 |
|------|------|------|
| `AugmentedLagrangianKnotCost::StageCost(x, u)` | `augmented_lagrangian_cost.cpp` | base_cost + AL 罚项 |
| `AugmentedLagrangianKnotCost::ConstraintAugmentedTerm(...)` | `augmented_lagrangian_cost.cpp` | 计算单个约束的 AL 项 |
| `AugmentedLagrangianKnotCost::UpdateDuals(x, u)` | `augmented_lagrangian_cost.cpp` | 更新 λ |
| `AugmentedLagrangianKnotCost::UpdatePenalties()` | `augmented_lagrangian_cost.cpp` | 更新 μ |
| `AugmentedLagrangianProblem::UnconstrainedSubproblem()` | `augmented_lagrangian_cost.cpp` | 返回无约束子问题 |

### 动力学

| 函数 | 文件 | 作用 |
|------|------|------|
| `KinematicBicycleModel::NextState(x, u, dt)` | `kinematic_bicycle_model.cpp` | 自行车模型递推 |
| `ILQRSolver::DynamicsJacobianState(x, u)` | `ilqr_solver.cpp` | 数值差分求 A_k |
| `ILQRSolver::DynamicsJacobianControl(x, u)` | `ilqr_solver.cpp` | 数值差分求 B_k |

### 约束

| 函数 | 文件 | 作用 |
|------|------|------|
| `ControlBoxConstraint::Evaluate(x, u)` | `control_box_constraint.cpp` | 控制量上下界 |
| `RoadBoundaryConstraint::Evaluate(x, u)` | `road_boundary_constraint.cpp` | 车道边界 |
| `SpeedLimitConstraint::Evaluate(x, u)` | `speed_limit_constraint.cpp` | 速度限制 |
| `MultiCircleVehicleObstacleConstraint::Evaluate(x, u)` | `multi_circle_vehicle_obstacle_constraint.cpp` | 碰撞检测 |
| `VehicleCircleApproximation::EvaluateWorldCenters(x)` | `vehicle_circle_approximation.cpp` | 车体圆心世界坐标 |

### 场景工厂

| 函数 | 文件 | 作用 |
|------|------|------|
| `CreateSingleStaticObstacleTestScenario()` | `demo_scenario.cpp` | 静态障碍物场景 |
| `CreateDynamicObstacleDemoScenario()` | `demo_scenario.cpp` | 动态障碍物场景 |
| `CreateDynamicObstacleSpeedStudyScenario(v)` | `demo_scenario.cpp` | 速度扫描场景 |
| `AddObstacleConstraints(scenario)` | `demo_scenario.cpp` | 添加碰撞约束 |

---

## F.9 一次完整求解的时间线

以 `phase8_autodrive_demo` 为例的典型执行时间线：

```
时间线 ──────────────────────────────────────────────────────────────────────►

  ┌──────┐   ┌─────────────────────────────────────────────────┐   ┌─────┐
  │ 场景  │   │            AL-iLQR 求解                         │   │ 输出 │
  │ 创建  │   │                                                 │   │ 结果 │
  └──┬───┘   └─────────────────────┬───────────────────────────┘   └──┬──┘
     │                             │                                   │
     │  CreateDemoScenario         │  Solve()                          │ Write
     │  ~1 ms                      │                                   │ CSV
     │                             │                                   │
     │  ┌── Outer 1 ──────────────────────────────────────┐            │
     │  │ ILQRSolver(subproblem)                          │            │
     │  │ ├─ Rollout (初始)                                │            │
     │  │ ├─ iter 1: Expand→Backward→Forward (α=1.0)     │            │
     │  │ ├─ iter 2: Expand→Backward→Forward (α=0.5)     │            │
     │  │ ├─ ...                                          │            │
     │  │ └─ iter 25: 收敛                                 │            │
     │  │ violation = 0.84                                │            │
     │  │ UpdateDuals, UpdatePenalties                    │            │
     │  └─────────────────────────────────────────────────┘            │
     │  ┌── Outer 2 ──────────────────────────────────────┐            │
     │  │ ILQRSolver (代价含更强的 AL 项)                  │            │
     │  │ ├─ iter 1..18: 求解                              │            │
     │  │ violation = 0.21                                │            │
     │  │ UpdateDuals                                     │            │
     │  └─────────────────────────────────────────────────┘            │
     │  ┌── Outer 3..5 ──────────────────────────────────┐             │
     │  │ ...                                            │             │
     │  │ violation 逐步下降:  0.04 → 0.008 → 0.0006     │             │
     │  │ violation < tolerance → converged!              │             │
     │  └─────────────────────────────────────────────────┘            │
     │                                                                 │
     ▼                                                                 ▼
  ~ 1 ms                        ~ 100-500 ms                       ~ 1 ms
```

---

## F.10 输出文件说明

### 轨迹 CSV（`WriteTrajectoryCsv`）

列头格式：`k,x0,x1,x2,x3,u0,u1`（状态和控制列名按维度编号）

对于自行车模型（state_dim=4, control_dim=2）：
- `k`：knot point 序号
- `x0,x1,x2,x3`：对应 x, y, yaw, velocity
- `u0,u1`：对应 acceleration, steering（最后一行 k=N 无控制值）

输出文件示例：
| 文件 | 说明 |
|------|------|
| `phase8_autodrive_trajectory.csv` | 静态避障优化后轨迹 |
| `phase11_dynamic_initial_trajectory.csv` | 动态避障优化前初始轨迹 |
| `phase11_dynamic_optimized_trajectory.csv` | 动态避障优化后轨迹 |

### 障碍物预测 CSV（`WriteObstaclePredictionCsv`）

列头格式：`k,cx,cy,radius`
- `k`：knot point 序号
- `cx,cy`：障碍物中心坐标
- `radius`：障碍物半径

输出文件：`phase11_dynamic_obstacle_prediction.csv`

### 外层迭代日志 CSV（`WriteALILQROuterLogCsv`）

列头格式：`outer_iteration,inner_iterations,base_cost,augmented_cost,max_violation,best_violation_so_far,max_penalty,penalty_updated`

输出文件：`phase11_dynamic_outer_log.csv`

这些 CSV 文件可以用 Python/matplotlib 或任何数据工具进行后续可视化分析。

---

## F.11 总结：从零到求解的完整路径

```
  拿到代码
     │
     ▼
  cmake + make → 编译所有目标
     │
     ▼
  运行 phase0..phase2 → 理解基础组件
     │
     ▼
  运行 phase3..phase4 → 理解 iLQR 内核
     │
     ▼
  运行 phase5..phase6 → 理解约束和 AL
     │
     ▼
  运行 phase7 → 看到完整 AL-iLQR 求解器工作
     │
     ▼
  运行 phase8 → 第一次看到自动驾驶避障轨迹! ★
     │
     ▼
  运行 phase11..phase13 → 动态障碍物、多场景对比
     │
     ▼
  运行 planner_gui / imgui_frontend → 交互式调参
     │
     ▼
  阅读 guide/ 文档 → 理解每一行代码背后的数学
     │
     ▼
  修改参数、添加约束、实现新模型 → 开始自己的项目!
```

> **注意**：本项目实现的是 AL-iLQR 主线算法，与参考论文存在多处差异（数值差分、条件性惩罚更新、iLQR 而非 DDP 等）。完整差异清单参见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)。
