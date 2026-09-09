# Chapter 7: 自动驾驶场景应用与碰撞建模

> **对应代码**：Phase 8-9  
> **目标**：将 AL-iLQR 应用于简化的自动驾驶避障 demo 场景，包括参考线跟踪、车道约束、碰撞检测

---

## 7.1 自动驾驶轨迹规划概述

### 基础概念

本章展示的是一个**简化的自动驾驶 demo 场景**，用于验证 AL-iLQR 的实际效果。场景采用直线参考线 + 单个圆形障碍物（静态或动态），是一个教学级别的 demo 而非完整的自动驾驶系统。

```
┌─────────────────────────────────────────────────────────┐
│          简化自动驾驶 Demo 场景                           │
│                                                         │
│  目标:                                                   │
│  ├── 跟踪直线参考线（简化的车道中心线）                    │
│  ├── 保持目标速度                                        │
│  ├── 控制量平稳（舒适性）                                 │
│  │                                                      │
│  约束:                                                   │
│  ├── 不超出车道边界                                      │
│  ├── 不超速                                              │
│  ├── 控制量在物理极限内（方向盘/加速度）                    │
│  └── 不与单个障碍物碰撞                                  │
│                                                         │
│  用 AL-iLQR 建模:                                        │
│    目标 → CostFunction (LaneTrackingCost)                │
│    约束 → ConstraintFunction (各种约束)                   │
│    动力学 → KinematicBicycleModel                        │
│                                                         │
│  当前场景限制:                                            │
│  ├── 参考线仅支持直线 (StraightReferenceLine)             │
│  ├── 场景工厂每次只封装单个圆形障碍物的轨迹序列            │
│  └── 障碍物运动模型仅支持匀速                             │
└─────────────────────────────────────────────────────────┘
```

---

## 7.2 参考线与车道跟踪代价

### 7.2.1 直线参考线

本项目使用直线参考线作为简化的车道中心线：

```
   参考线 (heading = 0, 沿 x 轴)
   ════════════════════════════════════►  x
         ↑
         │ lateral_error (横向偏差)
         │
         ● 车辆位置 (x, y)
```

`StraightReferenceLine` 计算车辆相对于参考线的误差：

```cpp
// include/autodrive/straight_reference_line.hpp
class StraightReferenceLine {
 public:
  StraightReferenceLine(double origin_x, double origin_y, double heading);

  double LongitudinalPosition(const Vector& state) const; // 纵向位置
  double LateralError(const Vector& state) const;          // 横向偏差
  double HeadingError(const Vector& state) const;          // 航向偏差
};
```

### 7.2.2 车道跟踪代价函数

`LaneTrackingCost` 以二次形式惩罚各种偏差：

$$
\ell_{\text{stage}} = \frac{1}{2} \left[ w_{\text{lat}} e_{\text{lat}}^2 + w_{\text{head}} e_{\text{head}}^2 + w_v (v - v_{\text{ref}})^2 + w_a a^2 + w_\delta \delta^2 \right]
$$

$$
\ell_{\text{terminal}} = \frac{1}{2} \left[ w_{\text{lon}}^f e_{\text{lon}}^2 + w_{\text{lat}}^f e_{\text{lat}}^2 + w_{\text{head}}^f e_{\text{head}}^2 + w_v^f (v - v_{\text{ref}})^2 \right]
$$

对应代码（`src/autodrive/lane_tracking_cost.cpp`）：

```cpp
double LaneTrackingCost::StageCost(const Vector& state, const Vector& control) const {
  const double lateral_error = reference_line_->LateralError(state);
  const double heading_error = reference_line_->HeadingError(state);
  const double speed_error = state(3) - target_speed_;

  return 0.5 * stage_lateral_weight_ * lateral_error * lateral_error       // 横向偏差
       + 0.5 * stage_heading_weight_ * heading_error * heading_error       // 航向偏差
       + 0.5 * stage_speed_weight_ * speed_error * speed_error             // 速度偏差
       + 0.5 * stage_accel_weight_ * control(0) * control(0)              // 加速度平滑
       + 0.5 * stage_steering_weight_ * control(1) * control(1);          // 转向平滑
}
```

---

## 7.3 道路边界约束

### 基础概念

车辆不能驶出车道。这是一个**不等式约束**：

$$
-w_{\text{road}} \leq e_{\text{lat}} \leq w_{\text{road}}
$$

转化为标准形式 $c \leq 0$：

$$
c = \begin{pmatrix} -w_{\text{road}} - e_{\text{lat}} \\ e_{\text{lat}} - w_{\text{road}} \end{pmatrix} \leq 0
$$

```
    车道上界 ─────────────────────
                                  
             可行区域               
                                  
    参考线  ═══════════════════════
                                  
             可行区域               
                                  
    车道下界 ─────────────────────
```

对应代码（`src/autodrive/road_boundary_constraint.cpp`）：

```cpp
Vector RoadBoundaryConstraint::Evaluate(const Vector& state, const Vector& control) const {
  const double lateral_error = reference_line_->LateralError(state);
  Vector values(2);
  values(0) = lateral_lower_bound_ - lateral_error;   // 下界约束
  values(1) = lateral_error - lateral_upper_bound_;    // 上界约束
  return values;
}
```

---

## 7.4 速度限制约束

$$
v_{\min} \leq v \leq v_{\max} \quad \Rightarrow \quad c = \begin{pmatrix} v_{\min} - v \\ v - v_{\max} \end{pmatrix} \leq 0
$$

---

## 7.5 碰撞约束建模

### 7.5.1 为什么不用矩形碰撞检测

```
  矩形碰撞检测:
  ┌─────────┐
  │ 车辆     │  ←→  ┌───┐ 障碍物
  │         │       │   │
  └─────────┘       └───┘

  问题:
  ① 矩形碰撞检测不可微 → 无法用基于梯度的优化
  ② 需要分离轴定理 (SAT) → 公式复杂
  ③ 旋转后计算更复杂
```

### 7.5.2 多圆覆盖车体

用多个圆覆盖车体，将碰撞检测转化为简单的**圆-圆距离计算**：

```
  车辆多圆近似 (collision_circle_count = 3):

      ┌───────────────────────────┐
      │   ○          ○          ○  │  ← 3个圆覆盖车体
      │ 后圆       中间圆      前圆 │
      └───────────────────────────┘
          ↑                     ↑
       rear_limit           front_limit

  圆心沿车体纵轴均匀分布
  半径 = 车宽/2
```

对应代码（`src/autodrive/vehicle_circle_approximation.cpp`）：

```cpp
VehicleCircleApproximation::VehicleCircleApproximation(VehicleBodyConfig config)
    : config_(std::move(config)) {
  const double radius = 0.5 * config_.width;                         // 半径 = 车宽/2
  const double rear_limit = -config_.rear_axle_to_rear + radius;     // 后圆心纵向位置
  const double front_limit = (config_.length - config_.rear_axle_to_rear) - radius; // 前圆心

  // 在 [rear_limit, front_limit] 之间均匀放置 N 个圆
  const double span = front_limit - rear_limit;
  const double step = span / (config_.collision_circle_count - 1);
  for (int i = 0; i < config_.collision_circle_count; ++i) {
    circles_.push_back(VehicleCircle{rear_limit + step * i, 0.0, radius});
  }
}
```

### 7.5.3 车体坐标到世界坐标的变换

圆心在车体坐标系下是固定的，需要根据车辆状态 $(x, y, \theta)$ 转换到世界坐标：

$$
\begin{pmatrix} p_x^{\text{world}} \\ p_y^{\text{world}} \end{pmatrix} = \begin{pmatrix} x \\ y \end{pmatrix} + \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} \begin{pmatrix} p_x^{\text{body}} \\ p_y^{\text{body}} \end{pmatrix}
$$

```
  车体坐标 → 世界坐标:

  世界坐标系:
        y ↑
          │     ○ 世界坐标系中的圆心
          │   ╱
          │ ╱ θ (yaw)
          ●───────→ x
        车辆位置

  旋转矩阵:
  R = [cos(θ) -sin(θ)]
      [sin(θ)  cos(θ)]
```

对应代码：

```cpp
std::vector<Vector> VehicleCircleApproximation::EvaluateWorldCenters(const Vector& state) const {
  const double x = state(0), y = state(1), yaw = state(2);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<Vector> centers;
  for (const auto& circle : circles_) {
    Vector center(2);
    // 旋转变换: 车体坐标 → 世界坐标
    center(0) = x + cos_yaw * circle.center_x_body - sin_yaw * circle.center_y_body;
    center(1) = y + sin_yaw * circle.center_x_body + cos_yaw * circle.center_y_body;
    centers.push_back(center);
  }
  return centers;
}
```

### 7.5.4 圆-圆碰撞约束公式

对于车体第 $i$ 个圆和一个障碍物，约束为：

$$
c_i = (r_{\text{obs}} + r_{\text{circle},i})^2 - \|p_{\text{circle},i} - p_{\text{obs}}\|^2 \leq 0
$$

```
  碰撞检测示意:

     ○ 车辆圆 (r_circle)        ○ 障碍物 (r_obs)
      \                         /
       \       距离 d          /
        ●─────────────────────●
        p_circle              p_obs

  安全条件: d > r_circle + r_obs
  约束: (r_circle + r_obs)² - d² ≤ 0

  为什么用距离的平方而不是距离:
  ① 避免开方运算 → 更快
  ② 距离平方对坐标可微 → 数值差分更稳定
```

对应代码（`src/autodrive/multi_circle_vehicle_obstacle_constraint.cpp`）：

```cpp
Vector MultiCircleVehicleObstacleConstraint::Evaluate(const Vector& state,
                                                      const Vector& control) const {
  const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);  // 车体圆心(世界坐标)
  Vector values(OutputDim());  // OutputDim = 车体圆的个数

  for (int i = 0; i < OutputDim(); ++i) {
    const double dx = centers[i](0) - obstacle_center_x_;
    const double dy = centers[i](1) - obstacle_center_y_;
    const double safe_distance = obstacle_radius_ + vehicle_geometry_.Circles()[i].radius;
    // c_i = safe_distance² - distance² ≤ 0
    values(i) = safe_distance * safe_distance - (dx * dx + dy * dy);
  }
  return values;
}
```

### 7.5.5 约束维度分析

```
  多圆 × 多障碍物 的约束维度:

  假设: 2个车体圆, 3个障碍物, horizon = 50

  每个 knot point, 每个障碍物:
    OutputDim = 2 (因为 2 个车体圆)

  每个 knot point 的障碍物约束总数:
    3 × 2 = 6

  所有 knot point 的总障碍物约束数:
    50 × 6 = 300

  加上其他约束 (控制框 4 + 道路边界 2 + 速度 2 = 8/knot):
    50 × 8 = 400

  总约束值: 300 + 400 = 700
  每个约束值有独立的 λ 和 μ !
```

### 7.5.6 非活跃约束自动消失

虽然约束很多，但大部分不等式约束在大多数 knot point 上是**非活跃的**（车辆离障碍物很远）。增广拉格朗日方法通过 $\max(0, \lambda + \mu c)$ 机制自动处理：

```
  约束 c_i 的 AL 项 = [max(0, λ_i + μ_i c_i)² - λ_i²] / (2μ_i)

  当 c_i << 0 (远离障碍物) 且 λ_i ≈ 0:
    max(0, 0 + μ_i · (负数)) = 0
    AL 项 = (0 - 0) / 2μ = 0  ← 零贡献！

  当 c_i ≈ 0 (接近障碍物):
    AL 项 > 0  ← 产生排斥力
```

---

## 7.6 动态障碍物与 Knot Point 的时间对齐

### 7.6.1 核心问题

规划问题是离散时间的。Horizon = 55，dt = 0.1s 意味着 56 个 knot point（k = 0, 1, ..., 55），总时长 5.5 秒。障碍物的约束必须在每个 knot point 上都成立。关键问题是：**每个 knot 上的障碍物位置从哪来？**

答案：**遍历每个 knot 的时间戳，用该时间戳去查询障碍物预测轨迹上对应的位置**。

### 7.6.2 时间戳与障碍物位置的映射

每个 knot point k 对应的物理时间是 $t_k = k \times \Delta t$：

```
Knot k:    0      1      2      3     ...    50     55
时间 t_k:  0.0s   0.1s   0.2s   0.3s  ...    5.0s   5.5s
                ↓      ↓      ↓      ↓             ↓      ↓
查询预测:  obs(0.0) obs(0.1) obs(0.2) obs(0.3) ... obs(5.0) obs(5.5)
```

对于匀速运动的障碍物，预测函数为：

$$
\mathbf{p}_{obs}(t_k) = \mathbf{p}_{obs,0} + \mathbf{v}_{obs} \times t_k = \mathbf{p}_{obs,0} + \mathbf{v}_{obs} \times k \times \Delta t
$$

实现代码为每个 knot 直接计算预测位置，生成一个长度为 H+1 = 56 的 `obstacle_centers` 数组：

```cpp
// src/autodrive/demo_scenario.cpp — 障碍物轨迹预测
const int H = scenario.base_problem->Horizon();    // 55
const double dt = scenario.base_problem->TimeStep(); // 0.1s
scenario.obstacle_centers.resize(H + 1);             // 56 个位置

for (int k = 0; k <= H; ++k) {
  // 用 knot k 的时间戳查询障碍物预测位置
  scenario.obstacle_centers[k] = ObstacleCenter(
      obstacle_center_x + obstacle_speed_x * k * dt,  // x 方向匀速预测
      obstacle_center_y);                               // y 方向不变
}
```

> **注意**：`obstacle_centers` 的长度恰好是 H+1 = 56，与 knot point **一一对应**，不存在插值问题。

### 7.6.3 静态障碍物是特例

静态障碍物就是速度为零的动态障碍物。当 `obstacle_speed_x = 0` 时，上述代码退化为所有 knot 使用相同位置：

```
静态 (v_obs = 0):
  k=0     k=1     k=2     ...   k=55
  (6,0)   (6,0)   (6,0)   ...   (6,0)   ← 所有 knot 位置相同

动态 (v_obs = 1.0 m/s):
  k=0     k=1     k=2     ...   k=55
  (6,0)   (6.1,0) (6.2,0) ...   (11.5,0) ← 每个 knot 位置不同
```

### 7.6.4 逐 Knot 创建约束对象

`AddObstacleConstraints` 遍历每个 stage k，用该时刻的预测位置 `obstacle_centers[k]` 创建一个**独立的**约束对象，并绑定到对应的 stage：

```cpp
// src/autodrive/demo_scenario.cpp — 逐 stage 绑定障碍物约束
void AddObstacleConstraints(AutodriveDemoScenario* scenario) {
  for (int k = 0; k < scenario->base_problem->Horizon(); ++k) {
    const double cx = scenario->obstacle_centers[k](0);  // 第 k 步的障碍物 x
    const double cy = scenario->obstacle_centers[k](1);  // 第 k 步的障碍物 y

    // 每个 stage 创建一个独立的约束对象，内含该时刻的障碍物位置
    scenario->constrained_problem->AddStageConstraint(
        k, std::make_shared<MultiCircleVehicleObstacleConstraint>(
               scenario->vehicle_geometry, cx, cy, scenario->obstacle_radius));
  }
}
```

**关键理解**：每个 stage 的约束对象是**独立的**，硬编码了该时刻障碍物的预测位置。求解器在优化第 k 步的状态 $x_k$ 时，使用的是约束 $C_k$ 中存储的 $(cx_k, cy_k)$，而不是某个全局的障碍物位置。

```
Stage 0 的约束: C_0 内含 obs_pos = (6.0, 0)   → 约束 x_0 与 (6.0, 0) 保持安全距离
Stage 1 的约束: C_1 内含 obs_pos = (6.1, 0)   → 约束 x_1 与 (6.1, 0) 保持安全距离
Stage 2 的约束: C_2 内含 obs_pos = (6.2, 0)   → 约束 x_2 与 (6.2, 0) 保持安全距离
  ...
Stage 50 的约束: C_50 内含 obs_pos = (11.0, 0) → 约束 x_50 与 (11.0, 0) 保持安全距离
```

### 7.6.5 仿真模式中的障碍物预测

在 MPC 仿真循环中，每次再规划时：

1. 计算障碍物的**当前位置**：$x_{obs}(t_{now}) = x_{obs,0} + v_{obs} \times t_{now}$
2. 以当前位置为起点，预测整个 horizon 内的轨迹：$x_{obs}(t_k) = x_{obs}(t_{now}) + v_{obs} \times k \times \Delta t$
3. 生成 56 个 `obstacle_centers`，创建约束

```
MPC 循环第 n 次规划 (sim_time = 2.0s):

障碍物当前位置: x = 6.0 + 1.0 × 2.0 = 8.0
规划 horizon 内的预测:
  k=0  → obs_x = 8.0  (t=2.0s)
  k=1  → obs_x = 8.1  (t=2.1s)
  k=2  → obs_x = 8.2  (t=2.2s)
  ...
  k=55 → obs_x = 13.5 (t=7.5s)
```

这使得规划器能够正确预判障碍物的运动趋势：
- **障碍物比自车快** → 预测显示障碍物越走越远 → 规划器选择直行跟车
- **障碍物比自车慢** → 预测显示会追上障碍物 → 规划器选择变道超车
- **障碍物迎面而来** → 预测显示障碍物快速接近 → 规划器提前避让

### 7.6.6 工程扩展：外部预测轨迹的接入

当前项目使用匀速模型直接计算预测，但在实际工程中，障碍物的预测轨迹通常来自独立的预测模块（如基于深度学习的轨迹预测）。接入方式完全相同：

```
外部预测模块输出（可能采样率不同）:
  t=0.0s → (8.0, 0.0)
  t=0.5s → (8.4, 0.1)
  t=1.0s → (8.9, 0.3)
  ...（共 10 个点）

转换为 obstacle_centers（需要插值到 knot 时间戳）:
  for k = 0 to H:
    t_k = k × dt
    obstacle_centers[k] = 在预测轨迹上按 t_k 进行线性插值
```

核心原则不变：**用 knot 的时间戳去索引（或插值）障碍物预测轨迹上的对应位置**。

---

## 7.7 场景组装

### 完整场景创建流程

```
    场景组装流程 (CreateScenarioSkeleton):

    ① 创建动力学模型
       KinematicBicycleModel(wheelbase)

    ② 创建参考线
       StraightReferenceLine(0, 0, 0)  (沿 x 轴)

    ③ 创建代价函数
       LaneTrackingCost(ref_line, target_speed, weights...)

    ④ 创建 OptimalControlProblem
       OCP(dynamics, cost, horizon=55, dt=0.1)

    ⑤ 创建 ConstrainedOptimalControlProblem
       包装 OCP

    ⑥ 逐步添加约束
       for k = 0 to N-1:
         ├── ControlBoxConstraint (加速度/转向限制)
         ├── RoadBoundaryConstraint (车道边界)
         └── SpeedLimitConstraint (速度限制)

    ⑦ 添加障碍物约束
       AddObstacleConstraints (逐步, 可能时变)

    ⑧ 设置初始状态和初始控制猜测

    ⑨ 配置求解器参数
       ALILQROptions {...}
```

---

## 7.8 求解示例

### 静态障碍物避障

```cpp
auto scenario = CreateSingleStaticObstacleTestScenario();
ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);
```

```
   典型求解结果:

   道路边界 ───────────────────────────────────────────
                         ╱──────╲
                       ╱          ╲         ← 绕过障碍物
   参考线 ═════════╱═══════●═══════╲═══════════
                ╱       障碍物        ╲
   道路边界 ──╱─────────────────────────╲──────
            起点                        终点
```

### 动态障碍物避障

```
   时间 t₀:           时间 t₃:
     ● 障碍物            ● 障碍物 (已移动)
     │                   │
   ══●══════════       ══●══════════
   车辆                 车辆 (已绕行)
```

---

## 7.9 MPC 仿真循环、Warm Start 与轨迹拼接

### 7.9.1 为什么需要 MPC 循环

Single Plan 模式只做一次规划：从起点到终点，一次算完。但真实驾驶是**持续决策**的过程：

- 环境在变化（障碍物在动）
- 模型有误差（执行不完美）
- 需要不断修正轨迹

MPC（Model Predictive Control）循环的核心思想：**规划 → 执行一小段 → 再规划 → 再执行 → ...**

```
MPC 循环 (Simulation 模式):

  ┌─────────────────────────────────────────────────┐
  │                                                 │
  │   ① 从当前状态 x_ego 出发                         │
  │   ② 构建规划问题 (障碍物预测、代价、约束)            │
  │   ③ 调用 AL-iLQR 求解 → 得到最优轨迹 u*[0..H-1]   │
  │   ④ 执行前 N 步 (replan_steps)                    │
  │   ⑤ 更新自车状态: x_ego ← x*(N)                   │
  │   ⑥ 更新仿真时间: t ← t + N × dt                  │
  │   └──→ 回到 ①                                     │
  │                                                 │
  └─────────────────────────────────────────────────┘

  时间线 (replan_steps = 5, dt = 0.1s):

  t=0.0s   t=0.5s   t=1.0s   t=1.5s   ...
  ├────────┤────────┤────────┤
  │ Plan 1  │ Plan 2  │ Plan 3  │  ...
  │ 求解    │ 求解    │ 求解    │
  │ 执行5步 │ 执行5步 │ 执行5步 │
```

### 7.9.2 Warm Start（热启动）

#### 问题

每次再规划时，AL-iLQR 需要一个**初始控制量序列**（initial guess）来启动优化。如果每次都用默认的零控制量，求解器需要从零开始搜索，收敛慢且可能产生不连续的轨迹。

#### 核心思想

利用上一次规划的解作为本次的初始猜测。由于两次规划之间只间隔了 `replan_steps` 步（通常 3-5 步），场景变化不大，上一次的解经过时间平移后是一个很好的初始猜测。

#### 控制量平移算法

假设上一轮求解得到的最优控制序列为：

$$
u^*_{\text{prev}} = [u_0, u_1, u_2, \ldots, u_{H-1}]
$$

自车执行了前 `N = replan_steps` 步后，这 N 个控制量已经被消耗。将剩余的控制量向前平移，末尾用最后一个有效值填充：

$$
u_{\text{warm}} = [\underbrace{u_N, u_{N+1}, \ldots, u_{H-1}}_{\text{前移的部分}}, \underbrace{u_{H-1}, u_{H-1}, \ldots, u_{H-1}}_{\text{填充 N 个}}]
$$

```
上一轮解:    [u₀, u₁, u₂, u₃, u₄, u₅, u₆, u₇, ...]
              ├──执行──┤
              (已消耗)

平移后:      [u₃, u₄, u₅, u₆, u₇, ..., u₇, u₇, u₇]
              ├──前移──────────────────┤├──填充──────┤
```

对应代码：

```cpp
// apps/rerun_imgui_frontend.cpp — ShiftControls
std::vector<Vector> ShiftControls(const std::vector<Vector>& controls, int shift) {
  const int n = static_cast<int>(controls.size());
  if (n == 0 || shift <= 0) return controls;

  std::vector<Vector> shifted;
  shifted.reserve(n);

  // 前移：跳过已执行的 shift 个控制量
  for (int i = shift; i < n; ++i) {
    shifted.push_back(controls[i]);
  }

  // 填充：用最后一个控制量重复填满
  while (static_cast<int>(shifted.size()) < n) {
    shifted.push_back(shifted.empty() ? Vector::Zero(2) : shifted.back());
  }
  return shifted;
}
```

#### 使用流程

```cpp
// 触发新一轮规划时：
std::vector<Vector> warm_ctrl;
if (use_warm_start && !previous_controls.empty()) {
  // 用平移后的上一轮控制量作为初始猜测
  warm_ctrl = ShiftControls(previous_controls, replan_steps);
}

// 求解时传入 warm start 控制量：
auto result = solver.Solve(ego_state, warm_ctrl.empty() ? default_controls : warm_ctrl);
```

#### 效果

| 指标 | 无 Warm Start | 有 Warm Start |
|------|---------------|---------------|
| 收敛速度 | 慢（从零搜索） | 快（在好的初始解附近） |
| 迭代次数 | 多 | 少 |
| 轨迹连续性 | 可能跳变 | 平滑过渡 |
| 求解时间 | ~200-400ms | ~50-150ms |

#### Warm Start 与初始解（Initial Guess）的关系

一个常见问题是：**有了 Warm Start，还需要计算初始解吗？**

答案是：**在某些情况下仍然需要**。Warm Start 并不能完全替代初始解的计算。

| 场景 | 是否需要计算初始解 | 原因 |
|------|-------------------|------|
| MPC 第一轮（冷启动） | **需要** | `previous_controls` 为空，没有可平移的上一轮解 |
| 用户在 UI 中关闭 Warm Start | **需要** | 显式选择不复用上一轮解 |
| 上一轮求解失败（未收敛） | **建议重算** | 失败的解质量差，平移后可能让本轮也难以收敛 |
| 场景剧变（如障碍物突然出现） | **建议重算** | 上一轮解在新场景下可能完全不可行，反而误导搜索方向 |
| MPC 后续轮（正常运行） | **不需要** | 平移后的上一轮解已经是很好的初始猜测 |

对应代码逻辑：

```cpp
// RunSimulationPlanningStep 内部
std::vector<Vector> init_controls;
if (use_warm_start && !warm_start_controls.empty()) {
  // 有上一轮解可用 → 用平移后的控制量
  init_controls = warm_start_controls;
} else {
  // 冷启动或关闭 Warm Start → 回退到场景默认初始控制量
  init_controls = scenario.initial_controls;
}
```

`scenario.initial_controls` 是由场景创建函数（`CreateSingleStaticObstacleTestScenario`）生成的默认初始猜测，通常是零控制量（加速度=0，转向=0）。这意味着初始 Rollout 是一条匀速直行轨迹。

> **总结**：Warm Start 是一种加速手段，不是替代品。每次规划都需要一个初始控制量序列，Warm Start 只是提供了一种更好的来源。当 Warm Start 不可用时，系统自动退回默认初始解。

#### 填充控制量的设计与局限

Warm Start 平移后，末尾被填充的 N 个控制量值得特别讨论。

**当前实现：重复最后一个有效控制量**

```
原始序列:  [u₀, u₁, ..., u₅₂, u₅₃, u₅₄]
                                     ↑ 最后一个有效值
平移后:    [u₅, u₆, ..., u₅₃, u₅₄, u₅₄, u₅₄, u₅₄, u₅₄, u₅₄]
                                    ├── 前移部分 ──┤├── 填充 N=5 ──┤
```

这 N 个填充值 $u_{H-1}$ 对应的是上一轮 horizon 的**最后一个控制量**。在一个已收敛的轨迹末端，车辆通常已趋近终端目标（接近匀速直行），所以最后一个控制量的加速度和转向角都接近零——这是一个合理的默认值。

**填充控制量是否需要考虑避障？**

简短的回答是：**当前实现不考虑，也不需要**。详细分析：

1. **填充段对应的是 horizon 末尾新增的时间段**：
   上一轮的 horizon 覆盖 $[t_0, t_0 + H \times dt]$，本轮 horizon 平移后覆盖 $[t_0 + N \times dt, t_0 + (H+N) \times dt]$。填充段对应 $[t_0 + H \times dt, t_0 + (H+N) \times dt]$ 这个"多出来"的尾巴。

2. **这个尾巴离障碍物通常很远**：
   horizon 末端（$H \times dt = 5.5$s 以后）对应的空间位置在当前自车前方很远处，大多数场景下障碍物已在 horizon 中段被避开。

3. **填充值只是初始猜测，不是最终解**：
   求解器会通过迭代优化整个控制序列（包括填充段），如果这些值导致约束违反，优化过程会自动修正。

4. **更精细的填充策略（可选改进）**：

   | 填充策略 | 复杂度 | 适用场景 |
   |---------|--------|---------|
   | 重复最后值（当前方案） | 最低 | 教学 demo，多数场景足够 |
   | 填零控制量 | 最低 | 匀速直行场景 |
   | 用 Rollout 做前向模拟 | 中等 | 需要更精确的 horizon 延伸 |
   | 针对新 horizon 段重新计算避障 | 高 | 障碍物密集、变化快的场景 |

   当前项目使用最简单的重复策略。对于教学 demo 中的单障碍物场景，这已经足够好。在更复杂的场景（多障碍物、高速行驶）中，可能需要更精细的策略。

### 7.9.3 Lattice 采样初始解模块

前面提到，Warm Start 不可用时（冷启动、场景剧变、首次规划），系统需要一个初始解。项目早期使用的是**硬编码**的"右绕回正"控制序列——无论障碍物在哪、车速多少，初始猜测都是同一组固定值。这种方式的局限性很明显：如果障碍物在右侧，硬编码的"右绕"反而会导致碰撞。

为此，项目实现了一个**多层 Lattice 采样模块**（`LatticeInitialGuess`），通过横向+纵向的组合采样，自动生成覆盖各种行为决策的**轨迹簇**，从中选出最优作为初始解。

#### 核心思想

```
                                      ┌── 停车 (v=0)
                    ┌── 左极限 ──────┤── 减速 (v=0.5×v_target)
                    │                 ├── 巡航 (v=v_target)
                    │                 └── 加速 (v=1.3×v_target)
                    │
                    │                 ┌── 停车
    起点 ──────────├── 左绕  ────────├── 减速
                    │                 ├── 巡航
                    │                 └── 加速
                    │
                    ├── 直行  ────────├── ...
                    │
                    ├── 右绕  ────────├── ...
                    │
                    └── 右极限 ──────├── ...

    横向 5 种 × 纵向 4 种 = 20 个候选节点/层
    多层 (2层): 20 × 20 = 400 条候选路径
    → 快速评估代价 → 选最优 → 转为控制量序列
```

横向采样决定了**绕行方向**（左绕/右绕/直行），纵向采样决定了**速度决策**（跟车/加速/减速/停车）。多层组合使得轨迹可以表达"先减速再左绕"等复合行为。

#### 算法详解

**Step 1：定义采样节点**

在 Frenet 坐标系下（参考线为纵轴），每个采样节点由 $(d, v)$ 定义：

- 横向偏移 $d$：默认 $\{-2.5, -1.5, 0, +1.5, +2.5\}$ 米
- 目标速度比例 $r$：默认 $\{0, 0.5, 1.0, 1.3\}$，乘以 `target_speed` 得到实际速度

**Step 2：多层 Lattice 图**

将 horizon $[0, T]$ 均分为 `num_layers` 段。每段的端点是一个"层"：

```
时间:    t=0        t=T/2       t=T
层:       Layer 0    Layer 1    Layer 2
           (起点)    (中间层)   (终端层)

         ● ─────── ● ─────── ●    (d=-2.5, v=0)
          \        / \       / \
           \      /   \     /   \
         ● ─────── ● ─────── ●    (d=0, v=v_target)
           /      \   /     \   /
          /        \ /       \ /
         ● ─────── ● ─────── ●    (d=+2.5, v=1.3×v_target)
                                    ...
```

每对相邻层的节点之间，用多项式曲线连接。

**Step 3：五次/四次多项式轨迹生成**

横向采用**五次多项式**（quintic polynomial），保证位置、速度、加速度的边界条件都满足：

$$
d(\tau) = a_0 + a_1 \tau + a_2 \tau^2 + a_3 \tau^3 + a_4 \tau^4 + a_5 \tau^5
$$

边界条件（6 个）：
- $d(0) = d_{\text{start}}, \; d'(0), \; d''(0)$ — 起点状态
- $d(T) = d_{\text{target}}, \; d'(T) = 0, \; d''(T) = 0$ — 终点（横向速度和加速度归零）

纵向采用**四次多项式**（quartic polynomial）：

$$
s(\tau) = b_0 + b_1 \tau + b_2 \tau^2 + b_3 \tau^3 + b_4 \tau^4
$$

边界条件（5 个）：
- $s(0) = s_{\text{start}}, \; s'(0) = v_{\text{start}}, \; s''(0) = 0$
- $s'(T) = v_{\text{target}}, \; s''(T) = 0$

这些多项式的系数通过解线性方程组得到（代码中使用 Eigen 求解 3×3 / 2×2 系统）。

**Step 4：动态规划选路**

使用 DP 在多层 lattice 图上搜索最优路径，避免枚举所有路径的指数爆炸：

$$
\text{cost}[l][j] = \min_{i} \left( \text{cost}[l-1][i] + \text{edge\_cost}(i \to j) \right)
$$

每条边的代价包括：
- **碰撞代价**：考虑动态障碍物（按时间偏移后的位置）和道路边界
- **横向偏差代价**：$w_d \sum_k d_k^2$
- **速度跟踪代价**：$w_v \sum_k (v_k - v_{\text{target}})^2$
- **控制平滑代价**：加速度和转向变化率的二次惩罚

**Step 5：转换为控制量**

最优路径的 Cartesian 轨迹 $(x_k, y_k, \theta_k, v_k)$ 通过逆运动学转换为控制量：

$$
a_k = \frac{v_{k+1} - v_k}{\Delta t}, \quad
\kappa_k = \frac{\theta_{k+1} - \theta_k}{v_k \cdot \Delta t}, \quad
\delta_k = \arctan(\kappa_k \cdot L)
$$

控制量最后被 clamp 到物理极限 $[a_{\min}, a_{\max}]$、$[\delta_{\min}, \delta_{\max}]$ 内。

#### 行为覆盖

| 横向偏移 | 纵向速度比 | 对应决策 |
|---------|-----------|---------|
| $d = 0$ | $r = 1.0$ | 直行巡航 |
| $d = -1.5$ | $r = 1.0$ | 左绕巡航 |
| $d = +1.5$ | $r = 1.0$ | 右绕巡航 |
| $d = 0$ | $r = 0$ | 停车 |
| $d = 0$ | $r = 0.5$ | 减速跟车 |
| $d = 0$ | $r = 1.3$ | 加速超车 |
| $d = -2.5$ | $r = 1.3$ | 左侧快速绕行 |
| ...  | ... | ... |

多层 lattice 还能表达**复合行为**，例如：
- Layer 1 → $(d=-1.5, v=0.5)$, Layer 2 → $(d=0, v=1.0)$：先减速左绕，再回到车道巡航
- Layer 1 → $(d=0, v=0)$, Layer 2 → $(d=+1.5, v=1.0)$：先停车等待，再右绕通过

#### 代码结构

```
include/autodrive/lattice_initial_guess.hpp  — 接口定义
src/autodrive/lattice_initial_guess.cpp      — 核心实现

关键类和函数：
├── LatticeConfig          — 采样参数（横向偏移、速度比、层数、代价权重）
├── LatticeObstacleInfo    — 障碍物信息
├── LatticeResult          — 返回最优控制量 + 统计信息
├── LatticeInitialGuess    — 主类
│   └── Generate()         — 生成最优初始解
├── SolveQuintic()         — 五次多项式求解
├── SolveQuartic()         — 四次多项式求解
├── SampleSegment()        — 多项式 → 离散轨迹点
├── EvaluateCandidate()    — 代价评估（碰撞+跟踪+平滑）
└── TrajectoryToControls() — 轨迹 → 控制量（逆运动学+clamp）
```

#### 使用方式

在 UI 的 "Initial Guess" 栏中勾选 **"Use Lattice Sampling"** 即可启用。启用后，`CreateSingleStaticObstacleTestScenario` 内部会调用 `LatticeInitialGuess::Generate` 替代硬编码初始解：

```cpp
if (config.use_lattice_init) {
  LatticeInitialGuess lattice(reference_line, wheelbase);
  auto result = lattice.Generate(initial_state, target_speed,
                                 road_half_width, vehicle_radius,
                                 obstacle_info, horizon, dt);
  scenario.initial_controls = result.controls;
} else {
  // 回退到硬编码初始猜测
  scenario.initial_controls = hardcoded_controls;
}
```

在 MPC 仿真循环中，当 Warm Start 不可用（首轮冷启动或关闭 Warm Start）时，系统会自动使用 lattice 生成的初始解。

#### 性能

| 配置 | 候选数量 | 耗时 |
|------|---------|------|
| 2 层, 5 横向 × 4 纵向 | 20 × 20 = 400 条路径 | ~2ms |
| 3 层, 5 横向 × 4 纵向 | 20 × 20 × 20 = 8000 条路径 | ~5ms |

DP 搜索的复杂度是 $O(L \times N^2)$（$L$ = 层数，$N$ = 每层节点数），远低于暴力枚举的 $O(N^L)$。

### 7.9.4 轨迹拼接（Trajectory Stitching）

#### 概念

**轨迹拼接**是 MPC 循环中连接前后两次规划结果的机制，确保自车行驶轨迹的连续性。

#### 工业界完整方案

在实车系统中，由于存在**控制跟踪误差**、**定位噪声**和**通信延迟**，车辆的实际位置往往偏离上一帧规划的轨迹。完整的轨迹拼接需要处理这些偏差：

```
上一帧规划轨迹:  ○───○───○───○───○───○───○
                               ↑
实际车辆位置:               ●  (存在跟踪偏差)
                               │
                  ┌────────────┘
                  ↓
步骤 1: 投影匹配
  在上一帧轨迹上找到离实际位置最近的点 (最近投影点)
                               ◎ ← 匹配点
                               │
步骤 2: 预瞄前进
  沿轨迹向前移动一个 "预瞄距离 d_preview" 或 "预瞄时间 t_preview"
  d_preview ≈ v × (t_plan + t_control)   (补偿规划耗时 + 控制延迟)
                               │
                               ↓
                                  ◆ ← 拼接起点
                                  │
步骤 3: 以拼接起点作为新规划的初始状态
  新规划从 ◆ 出发，保证新旧轨迹在此点平滑衔接
```

**预瞄距离的作用**：从匹配点到拼接起点的距离补偿了规划计算耗时和控制执行延迟。等新轨迹计算完毕并下发给控制器时，车辆恰好运动到拼接起点附近，实现无缝衔接。

工业界完整流程：

| 步骤 | 说明 | 需要的信息 |
|------|------|-----------|
| ① 获取实际状态 | 从定位模块获取车辆真实位姿 | GPS/IMU/里程计 |
| ② 投影匹配 | 在上一帧轨迹上找最近点 | 上一帧轨迹点序列 |
| ③ 预瞄延伸 | 沿轨迹前进 $d = v \times t_{\text{preview}}$ | 车速、预瞄时间 |
| ④ 取拼接起点 | 用预瞄点的 $(x, y, \theta, v)$ 作为规划起点 | — |
| ⑤ 生成新轨迹 | 新轨迹从拼接起点出发 | — |
| ⑥ 平滑拼接 | 旧轨迹的已执行段 + 新轨迹 = 完整路径 | — |

#### 当前项目的简化方案

当前项目**没有**独立的控制跟踪模块和定位误差，采用**完美执行假设**：自车状态直接取规划轨迹上的离散点。因此不需要投影匹配和预瞄距离，拼接过程大幅简化。

```
Plan 1 求解完成: [x₀, x₁, x₂, x₃, x₄, x₅, ..., x₅₅]
                       │
                       ├── 逐步执行 replan_steps = 5 步
                       │   ego_state ← x₁, x₂, x₃, x₄, x₅ (逐帧动画)
                       ↓
执行完毕后:  ego_state = x₅  (精确等于轨迹第 5 个点，无误差)
                       │
                       └── 直接用 x₅ 作为 Plan 2 的起点
                           (无需投影、无需预瞄)
```

连续多次规划的执行段首尾相连，形成完整轨迹：

```
Plan 1: [x₀ x₁ x₂ x₃ x₄ x₅ ... x₅₅]    ← 求解得到完整轨迹
              ├──执行──┤
              ↓ ↓ ↓ ↓ ↓
Plan 2:      [x₃' x₄' x₅' x₆' x₇' x₈' ... x₅₈']  ← 从 x₃ 出发重新规划
                        ├──执行──┤
                        ↓ ↓ ↓ ↓ ↓
Plan 3:                [x₆'' x₇'' x₈'' ...]

拼接后的历史轨迹:
  [x₀, x₁, x₂, x₃', x₄', x₅', x₆'', x₇'', ...]
   ├─Plan 1──┤├─Plan 2──────┤├─Plan 3──────...
```

#### 两种方案的对比

| 方面 | 工业界完整方案 | 当前项目简化方案 |
|------|--------------|----------------|
| 车辆状态来源 | 实车定位（有跟踪误差） | 直接取规划轨迹点（精确） |
| 跟踪误差 | 存在，必须补偿 | 不存在（完美执行假设） |
| 投影匹配 | 必须，在旧轨迹上找最近点 | 不需要 |
| 预瞄距离/时间 | 必须，补偿规划+控制延迟 | 不需要 |
| 拼接起点 | 旧轨迹上的预瞄点 | 直接是执行到的终点状态 |
| 新旧轨迹平滑性 | 靠预瞄 + Warm Start 共同保证 | 主要靠 Warm Start 保证 |
| 实现复杂度 | 高（需要投影、插值、延迟估计） | 低 |

> **如果要让本项目更接近实车**，可以扩展：加入带噪声的状态更新（模拟跟踪误差）、引入规划延迟补偿、实现基于投影 + 预瞄的完整拼接逻辑。

#### 实现代码

每次动画步进时，将当前位置记入历史，再推进到下一帧：

```cpp
// 动画步进 — 逐帧推进自车状态
if (anim_idx < anim_total) {
  anim_idx++;
  // 记录当前位置到历史
  history_x.push_back(ego_state(0));
  history_y.push_back(ego_state(1));
  // 推进到规划轨迹的下一步（完美执行假设）
  ego_state(0) = anim_x[anim_idx];
  ego_state(1) = anim_y[anim_idx];
  ego_state(2) = anim_yaw[anim_idx];
  ego_state(3) = anim_speed[anim_idx];
  sim_time += dt;
}
```

#### Warm Start 与轨迹拼接的关系

这两者是**互补的**，解决不同层面的问题：

```
┌──────────────────────────────────────────────────────┐
│  Warm Start:                                         │
│    解决 "优化初始猜测" 问题                             │
│    让求解器更快收敛、减少迭代次数                        │
│                                                      │
│  轨迹拼接:                                            │
│    解决 "执行连续性" 问题                               │
│    将多段规划结果拼成完整行驶路径                        │
│                                                      │
│  两者配合:                                            │
│    Warm Start 使新轨迹与旧轨迹的执行段自然对齐          │
│    → 拼接处过渡平滑                                    │
│                                                      │
│  如果关闭 Warm Start:                                 │
│    每次从默认控制量优化，新轨迹可能偏离旧轨迹            │
│    → 拼接处可能出现跳变                                │
└──────────────────────────────────────────────────────┘
```

### 7.9.5 完整 MPC 循环流程图

```
  ┌────────────────────────────────────────────────────────────────────┐
  │  MPC 仿真循环                                                       │
  │                                                                    │
  │  初始化:                                                            │
  │    ego_state ← (initial_x, initial_y, initial_yaw, initial_speed)  │
  │    sim_time ← 0                                                    │
  │    previous_controls ← 空                                          │
  │                                                                    │
  │  循环:                                                              │
  │  ┌──────────────────────────────────────────────────────────────┐  │
  │  │ ① 构建场景                                                    │  │
  │  │    config.initial_x/y/yaw/speed ← ego_state                 │  │
  │  │    config.obstacle_center_x ← obs₀ + v_obs × sim_time       │  │
  │  │    config.obstacle_speed_x ← v_obs (用于 horizon 内预测)      │  │
  │  │    config.target_longitudinal ← ego_x + v_target × 5.5s     │  │
  │  │                                                              │  │
  │  │ ② 准备初始猜测 (Warm Start)                                   │  │
  │  │    if warm_start && previous_controls 非空:                   │  │
  │  │      init_controls ← ShiftControls(previous_controls, N)    │  │
  │  │    else:                                                     │  │
  │  │      init_controls ← 默认控制量                               │  │
  │  │                                                              │  │
  │  │ ③ 求解                                                       │  │
  │  │    result ← AL-iLQR.Solve(ego_state, init_controls)         │  │
  │  │                                                              │  │
  │  │ ④ 保存控制量 (为下轮 Warm Start)                               │  │
  │  │    previous_controls ← result.controls[0..H-1]              │  │
  │  │                                                              │  │
  │  │ ⑤ 执行 + 轨迹拼接 (逐帧动画)                                   │  │
  │  │    for step = 1 to replan_steps:                             │  │
  │  │      history ← append(ego_state)                            │  │
  │  │      ego_state ← result.trajectory.State(step)              │  │
  │  │      sim_time += dt                                         │  │
  │  │                                                              │  │
  │  │ ⑥ 如果 running → 回到 ①                                      │  │
  │  └──────────────────────────────────────────────────────────────┘  │
  └────────────────────────────────────────────────────────────────────┘
```

### 7.9.6 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `replan_steps` | 5 | 每次规划后执行多少步再重新规划。值越小规划越频繁、响应越快，但计算量越大 |
| `use_warm_start` | true | 是否用上一轮解的平移作为初始猜测 |
| `target_speed` | 3.0 m/s | 自车目标速度 |
| `obstacle_speed_x` | 0.0 m/s | 障碍物沿 x 轴的速度，0 表示静态 |
| `target_longitudinal_position` | 动态计算 | 每轮自动设为 `ego_x + target_speed × 5.5s`，确保终端目标始终在前方 |

---

## 7.10 本章小结

```
┌──────────────────────────────────────────────────────────────┐
│              自动驾驶场景建模模块关系                            │
│                                                              │
│  StraightReferenceLine ──┐                                   │
│  (参考线)                │                                   │
│                          ├──→ LaneTrackingCost               │
│  VehicleBicycleConfig    │    (代价函数)                      │
│  (车辆参数)              │                                   │
│                          │                                   │
│  VehicleCircleApprox ────┤                                   │
│  (多圆车体)              │                                   │
│    └── 坐标变换          │                                   │
│                          │                                   │
│  Constraints:            │                                   │
│  ├── ControlBox ─────────┤                                   │
│  ├── RoadBoundary ───────┤                                   │
│  ├── SpeedLimit ─────────┤                                   │
│  └── MultiCircleObs ─────┤                                   │
│       └── 时变障碍物      │                                   │
│                          ▼                                   │
│  ConstrainedOCP ──→ ALILQRSolver ──→ 最优轨迹                │
│                         ↑ ↓                                  │
│  MPC 仿真循环:           │ │                                  │
│  ├── ShiftControls ──────┘ │ (Warm Start: 控制量平移)         │
│  ├── 执行 replan_steps ←───┘ (取前 N 步)                     │
│  ├── 历史轨迹拼接 (history)                                   │
│  └── 回到求解 → 循环                                          │
│                                                              │
│  场景工厂 (demo_scenario.cpp):                                │
│  ├── CreateSingleStaticObstacleTestScenario()                │
│  ├── CreateDynamicObstacleDemoScenario()                     │
│  └── CreateDynamicObstacleSpeedStudyScenario()               │
└──────────────────────────────────────────────────────────────┘
```
