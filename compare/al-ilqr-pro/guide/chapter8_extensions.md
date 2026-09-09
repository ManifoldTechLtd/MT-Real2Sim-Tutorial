# Chapter 8: 工程增强与扩展

> **对应代码**：Phase 10-13  
> **目标**：了解如何提升求解器的鲁棒性和实用性，包括参数配置、数值增强、动态障碍物、多场景对比等

---

## 8.1 车辆参数配置化

### 基础概念

硬编码参数不利于调试和复用。本项目将车辆参数从代码中解耦，放入 **INI 格式** 配置文件（`config/vehicle_bicycle_config.ini`）。

### 8.1.1 参数结构

```
┌─────────────────────────────────────┐
│        VehicleBicycleConfig          │
│                                     │
│  model (BicycleKinematicConfig):    │
│    ├── wheelbase: 2.8 m             │
│    ├── min/max_acceleration: ±1.0   │
│    ├── min/max_steering: ±0.45 rad  │
│    └── min/max_speed: [0, 6] m/s    │
│                                     │
│  body (VehicleBodyConfig):          │
│    ├── length: 4.6 m                │
│    ├── width: 1.9 m                 │
│    ├── rear_axle_to_rear: 1.0 m     │
│    └── collision_circle_count: 2    │
└─────────────────────────────────────┘
```

对应代码（`include/autodrive/vehicle_bicycle_config.hpp`）：

```cpp
struct BicycleKinematicConfig {
  double wheelbase = 2.8;
  double min_acceleration = -1.0;
  double max_acceleration = 1.0;
  double min_steering = -0.45;
  double max_steering = 0.45;
  double min_speed = 0.0;
  double max_speed = 6.0;
};

struct VehicleBodyConfig {
  double length = 4.6;
  double width = 1.9;
  double rear_axle_to_rear = 1.0;
  int collision_circle_count = 2;
};

struct VehicleBicycleConfig {
  BicycleKinematicConfig model;
  VehicleBodyConfig body;
};
```

### 8.1.2 配置加载

从 INI 文件加载参数，方便修改而不需要重新编译。INI 格式使用 `[section]` 分组和 `key = value` 键值对：

```ini
; config/vehicle_bicycle_config.ini
[bicycle_model]
wheelbase = 2.8
min_acceleration = -1.0
max_acceleration = 1.0
min_steering = -0.45
max_steering = 0.45
min_speed = 0.0
max_speed = 6.0

[vehicle_body]
length = 4.6
width = 1.9
rear_axle_to_rear = 1.0
collision_circle_count = 2
```

加载接口（`vehicle_bicycle_config.cpp` 内部使用自实现的 INI 解析器 `ParseIniFile()`）：

```cpp
VehicleBicycleConfig LoadVehicleBicycleConfig(const std::filesystem::path& path);
std::filesystem::path DefaultVehicleBicycleConfigPath();
// → 返回 config/vehicle_bicycle_config.ini 的路径
```

---

## 8.2 数值增强

### 基础概念

实际工程中，数值问题是影响求解器鲁棒性的主要因素。本节介绍项目中的关键数值增强措施。

### 8.2.1 差分步长选择

中心差分的精度依赖步长 $\epsilon$ 的选择：

$$
\frac{\partial f}{\partial x_i} \approx \frac{f(x + \epsilon e_i) - f(x - \epsilon e_i)}{2\epsilon}
$$

```
  ε 太大: 截断误差大 (二阶项残留)
  ε 太小: 舍入误差大 (浮点精度限制)
  
  理论最优: ε ≈ (机器精度)^{1/3} ≈ 6×10⁻⁶
  项目实际: ε = 1e-4 (兼顾稳定性)

  ┌──────────────────────────────┐
  │  总误差                       │
  │  │ \                  ╱      │
  │  │  \  截断误差      ╱       │
  │  │   \             ╱ 舍入误差 │
  │  │    ╲          ╱           │
  │  │     ╲  最优  ╱            │
  │  │      ╲──●──╱             │
  │  │                           │
  │  └────────────────→ ε        │
  │              ≈ 1e-4          │
  └──────────────────────────────┘
```

对应代码中的配置：

```cpp
struct ILQROptions {
  double derivative_epsilon = 1e-4;   // 数值差分步长
  // ...
};
```

### 8.2.2 Hessian 对称化

数值差分计算的 Hessian 可能不精确对称，累积误差会导致 Cholesky 分解失败：

$$
V_{xx} \leftarrow \frac{1}{2}(V_{xx} + V_{xx}^T)
$$

对应代码（在 backward pass 中）：

```cpp
Vxx = 0.5 * (Vxx + Vxx.transpose());  // 强制对称化
```

### 8.2.3 正则化的自适应调整

```
正则化参数自适应策略:

  参数:
  ├── regularization_init = 1e-5     (初始值)
  ├── regularization_min = 1e-8      (下限)
  ├── regularization_max = 1e5       (上限)
  ├── increase_factor = 10.0         (失败时的增大倍数)
  └── decrease_factor = 5.0          (成功时的减小倍数)

  流程:
  ┌─────────────────────────────────────────┐
  │ Backward Pass                           │
  │ ├── Cholesky(Quu + ρI) 成功?            │
  │ │   ├── YES → Forward Pass              │
  │ │   │   ├── 接受 → ρ /= 5 (减小)        │
  │ │   │   └── 拒绝 → ρ *= 10 (增大)       │
  │ │   └── NO → ρ *= 10 (增大), 重试        │
  │ └── ρ > ρ_max → 放弃                    │
  └─────────────────────────────────────────┘
```

---

## 8.3 动态障碍物扩展

### 基础概念

动态障碍物的位置随时间变化。在轨迹优化中，我们需要预测障碍物在每个 knot point 的位置。

### 8.3.1 匀速运动模型预测

最简单的预测：假设障碍物匀速运动：

$$
p_{\text{obs}}(t_k) = p_{\text{obs}}(0) + v_{\text{obs}} \cdot k \cdot \Delta t
$$

```
  动态障碍物预测:

  时间:  t₀     t₁     t₂     t₃     t₄
            ●      ●      ●      ●      ●
            │      ↓      ↓      ↓      ↓
            │    y₁=y₀+v·Δt               
            │             y₂=y₀+v·2Δt     
            │                    y₃=y₀+v·3Δt
            │                           y₄=y₀+v·4Δt
            
  每个 knot 的约束使用该时刻的预测位置!
```

对应代码：

```cpp
for (int k = 0; k <= scenario.base_problem->Horizon(); ++k) {
  const double time = k * scenario.base_problem->TimeStep();
  const double center_x = config.obstacle_center_x;
  const double center_y = config.obstacle_initial_y + config.obstacle_speed_y * time;
  scenario.obstacle_centers.push_back(ObstacleCenter(center_x, center_y));
}
```

### 8.3.2 速度扫描研究

项目提供了不同障碍物速度下的求解对比（`CreateDynamicObstacleSpeedStudyScenario`），展示求解器在不同场景下的行为：

```
  障碍物速度扫描:

  v = 0.0  (静态) → 标准绕行
  v = 0.5  (慢速) → 小幅让行
  v = 1.0  (中速) → 中幅让行
  v = 2.0  (快速) → 大幅让行或等待
```

```cpp
AutodriveDemoScenario CreateDynamicObstacleSpeedStudyScenario(double obstacle_speed_y) {
  DynamicObstacleScenarioConfig config;
  config.obstacle_initial_y = -0.35;
  config.obstacle_speed_y = obstacle_speed_y;   // 可变速度参数
  config.obstacle_radius = 0.7;
  // ...
  return CreateDynamicObstacleDemoScenario(config);
}
```

---

## 8.4 场景配置系统

### 8.4.1 场景工厂模式

项目使用工厂函数创建不同场景，方便测试和对比：

```
  场景工厂:

  CreateSingleStaticObstacleTestScenario()
    └── 静态障碍物避障 (最基本场景)

  CreateDynamicObstacleDemoScenario()
    └── 动态障碍物避障 (标准配置)

  CreateDynamicObstacleDemoScenario(config)
    └── 动态障碍物避障 (自定义配置)

  CreateDynamicObstacleSpeedStudyScenario(speed)
    └── 障碍物速度参数研究
```

### 8.4.2 初始控制猜测

好的初始猜测可以显著加速收敛。项目对不同场景提供了启发式初始控制：

```cpp
// 静态障碍物场景: 提供"绕行"方向的初始猜测
for (int k = 0; k < horizon; ++k) {
  scenario.initial_controls[k](0) = (k < 8) ? 0.2 : 0.0;   // 先加速
  if (k < 15) {
    scenario.initial_controls[k](1) = 0.28;    // 先左转
  } else if (k < 30) {
    scenario.initial_controls[k](1) = -0.24;   // 再右转
  } else {
    scenario.initial_controls[k](1) = 0.08;    // 最后回正
  }
}
```

```
  初始控制猜测的效果:

  差的初始猜测 (全零):     好的初始猜测 (启发式):
  ════════════●═══════     ════╲═══════╱════════
            障碍物               ╲  ●  ╱
                                  ╲╱
  可能需要 20+ 外层迭代      通常 5-8 外层迭代即可收敛
```

---

## 8.5 求解器参数调优指南

### 8.5.1 内层 iLQR 参数

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `max_iterations` | 35 | 每次外层迭代的最大内层步数 |
| `cost_tolerance` | 1e-5 | 代价变化收敛阈值 |
| `derivative_epsilon` | 1e-4 | 数值差分步长 |
| `regularization_init` | 1e-5 | 初始正则化 |
| `line_search_max_iterations` | 10 | 线搜索最大尝试次数 |

### 8.5.2 外层 AL 参数

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `max_outer_iterations` | 18 | 最大外层迭代次数 |
| `constraint_tolerance` | 1e-1 | 约束满足精度 |
| `initial_penalty` | 15-20 | 初始 $\mu$ |
| `penalty_scaling` | 5 | $\phi$，惩罚增长速度 |
| `penalty_update_ratio` | 0.4 | 进展判断阈值 |
| `max_penalty` | 1e8 | $\mu$ 上限 |

### 8.5.3 调参建议

```
  常见问题和解决方案:

  约束不收敛 ──→ 增大 initial_penalty 或 penalty_scaling
  
  数值不稳定 ──→ 增大 regularization_init 或 derivative_epsilon
  
  收敛太慢 ──→ 减小 cost_tolerance, 增大 max_iterations
  
  iLQR 不收敛 ──→ 改善初始控制猜测
  
  μ 爆炸 ──→ 减小 penalty_scaling, 增大 penalty_update_ratio
```

---

## 8.6 项目完整架构回顾

```
┌──────────────────────────────────────────────────────────────────┐
│                    MY-AL-iLQR 项目完整架构                        │
│                                                                  │
│  核心层 (core/):                                                 │
│  ├── types.hpp (Vector, Matrix)                                  │
│  └── trajectory.hpp (Trajectory: states + controls)              │
│                                                                  │
│  动力学层 (dynamics/):                                            │
│  ├── dynamics_model.hpp (接口)                                    │
│  ├── linear_point_mass_model.hpp                                 │
│  ├── unicycle_model.hpp                                          │
│  └── kinematic_bicycle_model.hpp                                 │
│                                                                  │
│  代价函数层 (cost/):                                              │
│  ├── cost_function.hpp (接口)                                    │
│  └── quadratic_cost.hpp                                          │
│                                                                  │
│  约束层 (constraints/):                                           │
│  ├── constraint_function.hpp (接口)                               │
│  ├── control_box_constraint.hpp                                  │
│  └── terminal_goal_constraint.hpp                                │
│                                                                  │
│  问题封装层 (problems/):                                          │
│  ├── optimal_control_problem.hpp (无约束)                         │
│  └── constrained_optimal_control_problem.hpp (带约束)             │
│                                                                  │
│  求解器层:                                                        │
│  ├── lqr/finite_horizon_lqr_solver.hpp (线性 LQR)                │
│  ├── ilqr/ilqr_solver.hpp (非线性无约束 iLQR)                    │
│  └── al/                                                         │
│      ├── augmented_lagrangian_cost.hpp (AL 代价转换)              │
│      └── al_ilqr_solver.hpp (完整带约束求解器)                    │
│                                                                  │
│  自动驾驶应用层 (autodrive/):                                     │
│  ├── straight_reference_line.hpp (参考线)                         │
│  ├── lane_tracking_cost.hpp (车道跟踪代价)                        │
│  ├── vehicle_bicycle_config.hpp (车辆参数)                        │
│  ├── vehicle_circle_approximation.hpp (多圆车体)                  │
│  ├── road_boundary_constraint.hpp (道路边界)                      │
│  ├── speed_limit_constraint.hpp (速度限制)                        │
│  ├── multi_circle_vehicle_obstacle_constraint.hpp (碰撞约束)      │
│  └── demo_scenario.hpp (场景工厂)                                │
│                                                                  │
│  数据流:                                                          │
│  场景配置 → ConstrainedOCP → AugmentedLagrangianProblem           │
│    → ILQRSolver (内层) ↔ ALILQRSolver (外层) → 最优轨迹          │
└──────────────────────────────────────────────────────────────────┘
```

---

## 8.7 本章小结与展望

### 已实现的功能

```
  ✓ 基础设施: Trajectory, DynamicsModel, CostFunction
  ✓ 线性求解: FiniteHorizonLQRSolver (Riccati 递推)
  ✓ 非线性求解: ILQRSolver (展开 + backward + forward + 正则化)
  ✓ 约束框架: ConstraintFunction + ConstrainedOCP
  ✓ 约束求解: AugmentedLagrangian + ALILQRSolver
  ✓ 自动驾驶场景: 碰撞避障 + 车道跟踪
  ✓ 工程增强: 参数配置化 + 数值增强
  ✓ 动态障碍物: 时变约束 + 速度研究
```

### 与论文的差异汇总

本项目实现的是 AL-iLQR 主线算法。以下是本项目与 `AL_iLQR_Tutorial.pdf` 和 `ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf` 两篇论文的**全部已知差异**：

#### 1. ALTRO 高级特性（未实现）

| ALTRO 特性 | 论文描述 | 本项目状态 |
|------------|---------|-----------|
| Square-root backward pass | 用 Cholesky 因子代替完整 $V_{xx}$ 矩阵，提升数值稳定性 | 未实现，使用标准矩阵形式 |
| Infeasible-state initialization | 允许初始轨迹不满足动力学约束，通过虚拟控制修正 | 未实现，初始轨迹必须通过 Rollout 生成 |
| Projection polishing | AL-iLQR 收敛后，用投影方法精确满足约束 | 未实现 |

#### 2. 导数计算方式

| 论文做法 | 本项目做法 | 影响 |
|---------|-----------|------|
| 解析推导动力学 Jacobian $A_k, B_k$ | 数值中心差分（步长 $\epsilon = 10^{-4}$） | 精度受 $\epsilon$ 影响，计算量更大，但无需手推公式 |
| 解析推导约束 Jacobian $c_x, c_u$ 并显式展开 AL 梯度 | AL 罚项包进 `StageCost()` 后，对整体代价统一做数值差分 | 数学等价，但新增约束只需实现 `Evaluate()` |
| iLQR 或 full DDP（含动力学二阶展开 $f_{xx}$ 等） | 仅 iLQR（一阶动力学线性化），无 $f_{xx}, f_{xu}, f_{uu}$ | DDP 在强非线性处收敛更快，但实现复杂度高 |

#### 3. 惩罚更新策略

| 论文标准做法 | 本项目做法 | 详见 |
|-------------|-----------|------|
| 每次外层迭代无条件 $\mu \leftarrow \phi \cdot \mu$ | 条件性更新：只在进展不足时增大 $\mu$（两个判据，含 `max(ratio, 0.95)` 停滞检测） | Chapter 6 §6.4 |

这是项目自己的工程启发式（heuristic），优点是避免 $\mu$ 增长过快导致子问题数值病态，缺点是引入了额外的可调参数（`penalty_update_ratio`）。

#### 4. 内层求解器生命周期

| 论文隐含做法 | 本项目做法 | 影响 |
|-------------|-----------|------|
| 内层求解器跨外层迭代复用工作空间 | 每次外层迭代重新创建 `ILQRSolver` 对象（见 `al_ilqr_solver.cpp` 第 61 行） | 功能正确，但存在不必要的内存分配开销 |

#### 5. AL 约束公式形式

| 论文（ALTRO） | 本项目 | 等价性 |
|--------------|--------|-------|
| Conic 形式：$\frac{1}{2\rho}(\|\Pi_{K^*}(\lambda - \rho c)\|^2 - \|\lambda\|^2)$ | 标准形式：等式 $\lambda c + \frac{\mu}{2}c^2$；不等式 $\frac{\max(0, \lambda+\mu c)^2 - \lambda^2}{2\mu}$ | 数学等价，符号约定不同（$\lambda$ 的正负方向） |

### 可能的扩展方向

| 方向 | 说明 |
|------|------|
| 解析 Jacobian | 替代数值差分，提升精度和效率 |
| 更复杂的动力学 | 动力学自行车模型、带轮胎力模型 |
| 曲线参考线 | 处理弯道、交叉口 |
| 多障碍物 | 多个静态/动态障碍物的混合场景 |
| 实时求解 | 热启动 + 缩短时域 |
| MPC 框架 | 滚动时域在线规划 |
| 自动微分 | 替代数值差分，精确且高效 |
