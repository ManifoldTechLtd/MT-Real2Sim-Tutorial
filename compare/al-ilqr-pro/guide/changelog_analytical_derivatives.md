# 更新记录：解析求导（Analytical Derivatives）全面支持

**版本**：v2.0 &nbsp;|&nbsp; **日期**：2026-04-12 &nbsp;|&nbsp; **类别**：性能优化 · 核心架构升级

---

## 摘要

本次更新为 AL-iLQR 求解器引入了**完整的解析求导能力**，覆盖动力学 Jacobian、所有代价函数的梯度/Hessian、以及全部约束（`PolygonCollisionConstraint` 除外）的 Jacobian。用户可通过 UI 界面**实时切换**数值差分与解析求导模式，便于对比验证。

### 性能提升

| 指标 | 数值差分（旧） | 解析求导（新） | 提升倍数 |
|------|---------------|---------------|---------|
| 单次规划耗时 | ~150 ms | ~9.6 ms | **15.6×** |

> 测试环境：80 步时域、3 个多圆覆盖圆约束 + 道路边界 + 速度限制 + 控制量边界。

---

## 一、为什么快了这么多？

### 1.1 旧方案的瓶颈：中心差分的 $O(n^2)$ 调用量

旧版对每个时间步的每个求导项都使用中心差分，以 $n=4$（状态维度）、$m=2$（控制维度）计算：

| 求导项 | 每步差分调用次数 |
|--------|:---------------:|
| $A = \partial f/\partial x$ | $2n = 8$ 次 `NextState` |
| $B = \partial f/\partial u$ | $2m = 4$ 次 `NextState` |
| $\ell_{xx}$（Hessian） | $4n^2 = 64$ 次 `StageCost` |
| $\ell_{uu}$ | $4m^2 = 16$ 次 `StageCost` |
| $\ell_{ux}$（交叉项） | $4nm = 32$ 次 `StageCost` |
| $\ell_x, \ell_u$（梯度） | $2(n+m) = 12$ 次 `StageCost` |

**每步合计**：12 次动力学 + 124 次代价函数（每次代价函数内部还会遍历所有约束的 `Evaluate`）。80 步时域下，每次 iLQR 迭代约需 **10,000+ 次函数调用**。

### 1.2 新方案：零差分调用

解析求导直接通过闭式公式计算所有导数，**无循环差分、无重复函数调用**。特别是：

- **代价 Hessian**：从 $4n^2 = 64$ 次函数调用 → 1 次矩阵外积运算
- **约束 AL 项**：从隐式标量差分 → 显式 $J^\top \operatorname{diag}(\mu)\, J$ Gauss-Newton 近似
- **动力学 Jacobian**：从 12 次 `NextState` → 1 次三角函数计算

---

## 二、架构设计：可切换的求导模式

### 2.1 DerivativeMode 枚举

```cpp
enum class DerivativeMode {
  kFiniteDifference,   // 保留原有中心差分（兜底/对照）
  kAnalytical,         // 解析求导（新增）
};
```

在 `ILQROptions` 中新增 `derivative_mode` 字段，默认为 `kFiniteDifference`（向后兼容）。

### 2.2 三层可选接口

在 `DynamicsModel`、`CostFunction`、`ConstraintFunction` 三个基类中统一新增可选的解析求导虚方法：

**动力学模型** — `DynamicsModel`：
```cpp
virtual bool HasAnalyticalJacobian() const { return false; }
virtual Matrix JacobianState(const Vector& state, const Vector& control, double dt) const;
virtual Matrix JacobianControl(const Vector& state, const Vector& control, double dt) const;
```

**代价函数** — `CostFunction`：
```cpp
virtual bool HasAnalyticalDerivatives() const { return false; }
virtual CostExpansion AnalyticalStageCostExpansion(const Vector& state, const Vector& control) const;
virtual std::pair<Vector, Matrix> AnalyticalTerminalCostExpansion(const Vector& state) const;
```

**约束函数** — `ConstraintFunction`：
```cpp
virtual bool HasAnalyticalJacobian() const { return false; }
virtual Matrix JacobianState(const Vector& state, const Vector& control) const;
virtual Matrix JacobianControl(const Vector& state, const Vector& control) const;
```

### 2.3 ILQRSolver 自动分派

`ILQRSolver` 在计算导数时检查 `derivative_mode` 和对应组件的 `HasAnalytical*()` 标志：
- 若模式为 `kAnalytical` **且** 组件支持解析求导 → 调用解析方法
- 否则 → 回退到中心差分

这意味着即使某个约束（如 `PolygonCollisionConstraint`）没有实现解析 Jacobian，系统仍能正常工作——对该约束对应的 AL 代价项仍使用差分。

### 2.4 AL 罚项解析展开

`AugmentedLagrangianKnotCost` 直接利用约束的解析 Jacobian 组装 AL 项的梯度和 Hessian，避免了对标量 AL 代价的全维度差分：

**不等式约束展开**（Gauss-Newton 近似）：

$$
\frac{\partial \varphi}{\partial x} = J_x^\top \max\!\bigl(0,\;\lambda + \mu \odot c\bigr)
$$

$$
\frac{\partial^2 \varphi}{\partial x^2} \approx J_x^\top \operatorname{diag}(I_{\text{active}} \odot \mu)\, J_x
$$

### 2.5 UI 实时切换

在 ImGui 前端的 **Planning Config** 面板中新增 **Derivative Mode** 下拉框，用户可实时在 `Finite Difference` 和 `Analytical` 之间切换，立即对比求解耗时和轨迹质量。

---

## 三、已实现解析求导的组件清单

### 3.1 动力学模型

| 组件 | 解析公式 | 说明 |
|------|---------|------|
| `KinematicBicycleModel` | A (4×4), B (4×2) | 前向欧拉离散化后的 Jacobian，含 sin/cos/tan |

### 3.2 代价函数

| 组件 | 解析项 | 说明 |
|------|-------|------|
| `QuadraticCost` | lx, lu, lxx, luu, lux | 二次型闭式解 |
| `LaneTrackingCost` | lx, lu, lxx, luu, lux + 终端 | 横向/航向/速度误差的外积分解 |
| `GuidanceTrackingCost` | lx, lxx + 终端 | 含纵向投影终端模式 |
| `ControlRateCost` | lu, luu | 仅控制量差分项 |
| `CompositeCostFunction` | 聚合子项 | 自动聚合所有子代价的解析展开 |

### 3.3 约束函数

| 组件 | Jacobian 维度 | 说明 |
|------|:------------:|------|
| `ControlBoxConstraint` | (4×4), (4×2) | 常数矩阵 ±I |
| `SpeedLimitConstraint` | (2×4), (2×2) | 仅 v 分量非零 |
| `TerminalGoalConstraint` | (n×n) | 单位矩阵 |
| `RoadBoundaryConstraint` | (2×4), (2×2) | 依赖参考线航向 h |
| `CircularObstacleConstraint` | (1×4), (1×2) | -2[dx, dy, 0, 0] |
| `MultiCircleRoadBoundaryConstraint` | (2k×4), (2k×2) | 含刚体变换 Jacobian |
| `MultiCircleVehicleObstacleConstraint` | (k×4), (k×2) | 距离对圆心 + 刚体变换链式法则 |
| `LSEPolygonObstacleConstraint` | (k×4), (k×2) | LSE softmax 梯度 + 刚体变换 |

### 3.4 未实现解析求导的组件

| 组件 | 原因 |
|------|------|
| `PolygonCollisionConstraint` | 基于 GJK 的凸多边形精确距离在顶点/边切换处不可微 |

该约束在 `kAnalytical` 模式下会自动回退到中心差分。

---

## 四、代码变更文件清单

### 新增文件

| 文件 | 用途 |
|------|------|
| `src/dynamics/dynamics_model.cpp` | 基类默认实现（抛异常兜底） |
| `src/cost/cost_function.cpp` | 基类默认实现 |
| `guide/derivative_architecture.md` | 求导架构分析与公式推导文档 |

### 修改文件

**接口层**（新增虚方法）：
- `include/dynamics/dynamics_model.hpp`
- `include/cost/cost_function.hpp` — 含新增 `CostExpansion` 结构体
- `include/constraints/constraint_function.hpp`
- `include/ilqr/ilqr_solver.hpp` — 新增 `DerivativeMode` 枚举

**动力学解析 Jacobian**：
- `include/dynamics/kinematic_bicycle_model.hpp`
- `src/dynamics/kinematic_bicycle_model.cpp`

**代价函数解析求导**：
- `include/cost/quadratic_cost.hpp` / `src/cost/quadratic_cost.cpp`
- `include/cost/composite_cost_function.hpp` / `src/cost/composite_cost_function.cpp`
- `include/autodrive/lane_tracking_cost.hpp` / `src/autodrive/lane_tracking_cost.cpp`
- `include/autodrive/guidance_tracking_cost.hpp` / `src/autodrive/guidance_tracking_cost.cpp`
- `include/autodrive/control_rate_cost.hpp` / `src/autodrive/control_rate_cost.cpp`

**约束解析 Jacobian**：
- `include/constraints/control_box_constraint.hpp` / `src/constraints/control_box_constraint.cpp`
- `include/constraints/terminal_goal_constraint.hpp` / `src/constraints/terminal_goal_constraint.cpp`
- `include/autodrive/speed_limit_constraint.hpp` / `src/autodrive/speed_limit_constraint.cpp`
- `include/autodrive/road_boundary_constraint.hpp` / `src/autodrive/road_boundary_constraint.cpp`
- `include/autodrive/circular_obstacle_constraint.hpp` / `src/autodrive/circular_obstacle_constraint.cpp`
- `include/autodrive/multi_circle_road_boundary_constraint.hpp` / `src/autodrive/multi_circle_road_boundary_constraint.cpp`
- `include/autodrive/multi_circle_vehicle_obstacle_constraint.hpp` / `src/autodrive/multi_circle_vehicle_obstacle_constraint.cpp`
- `include/autodrive/lse_polygon_obstacle_constraint.hpp` / `src/autodrive/lse_polygon_obstacle_constraint.cpp`
- `include/autodrive/convex_polygon.hpp` / `src/autodrive/convex_polygon.cpp` — 新增 `LogSumExpGradient`

**AL 层解析展开**：
- `include/al/augmented_lagrangian_cost.hpp` / `src/al/augmented_lagrangian_cost.cpp`

**求解器分派逻辑**：
- `src/ilqr/ilqr_solver.cpp`

**场景配置与 UI**：
- `include/autodrive/demo_scenario.hpp` — `StaticObstacleScenarioConfig` 新增 `derivative_mode`
- `src/autodrive/demo_scenario.cpp` — 传递到 `solver_options`
- `apps/rerun_imgui_frontend.cpp` — Planning Config 面板新增 Derivative Mode 下拉框

**构建系统**：
- `CMakeLists.txt` — 新增 `dynamics_model.cpp`、`cost_function.cpp`

---

## 五、解析求导公式推导

以下对除 `PolygonCollisionConstraint` 外的所有组件给出解析导数公式。

状态定义 $\mathbf{x} = [p_x,\; p_y,\; \theta,\; v]^\top$，控制定义 $\mathbf{u} = [a,\; \delta]^\top$（加速度、前轮转角）。
参考线航向 $h$、原点 $(o_x, o_y)$、轮距 $L$。

### 5.1 KinematicBicycleModel — Jacobian A, B

动力学方程（前向欧拉）：

$$
\begin{aligned}
f_0 &= p_x + \Delta t \cdot v \cos\theta \\
f_1 &= p_y + \Delta t \cdot v \sin\theta \\
f_2 &= \theta + \Delta t \cdot \frac{v \tan\delta}{L} \\
f_3 &= v + \Delta t \cdot a
\end{aligned}
$$

$A = \dfrac{\partial f}{\partial x}$（$4 \times 4$）：

$$
A = \begin{bmatrix}
1 & 0 & -\Delta t\, v \sin\theta & \Delta t \cos\theta \\
0 & 1 & \Delta t\, v \cos\theta & \Delta t \sin\theta \\
0 & 0 & 1 & \dfrac{\Delta t \tan\delta}{L} \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$B = \dfrac{\partial f}{\partial u}$（$4 \times 2$）：

$$
B = \begin{bmatrix}
0 & 0 \\
0 & 0 \\
0 & \dfrac{\Delta t\, v}{L \cos^2\delta} \\
\Delta t & 0
\end{bmatrix}
$$

### 5.2 QuadraticCost

**Stage cost：**

$$
\ell = \tfrac{1}{2}(x - x_r)^\top Q\,(x - x_r) + \tfrac{1}{2}(u - u_r)^\top R\,(u - u_r)
$$

$$
\ell_x = Q(x - x_r),\quad \ell_u = R(u - u_r),\quad \ell_{xx} = Q,\quad \ell_{uu} = R,\quad \ell_{ux} = 0
$$

**Terminal cost：**

$$
\ell_f = \tfrac{1}{2}(x - x_r)^\top Q_f\,(x - x_r)
$$

$$
\ell_x = Q_f(x - x_r),\quad \ell_{xx} = Q_f
$$

### 5.3 LaneTrackingCost

定义中间变量：

$$
\begin{aligned}
e_{\text{lat}} &= -\sin h \cdot (p_x - o_x) + \cos h \cdot (p_y - o_y) \\
e_{\text{hdg}} &= \theta - h \quad (\text{归一化到} [-\pi,\pi]) \\
e_{\text{spd}} &= v - v_{\text{target}}
\end{aligned}
$$

对状态的 Jacobian：

$$
\frac{\partial e_{\text{lat}}}{\partial x} = \begin{bmatrix} -\sin h & \cos h & 0 & 0 \end{bmatrix}, \quad
\frac{\partial e_{\text{hdg}}}{\partial x} = \begin{bmatrix} 0 & 0 & 1 & 0 \end{bmatrix}, \quad
\frac{\partial e_{\text{spd}}}{\partial x} = \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix}
$$

**Stage cost：**

$$
\ell = \tfrac{1}{2} w_l\, e_{\text{lat}}^2 + \tfrac{1}{2} w_h\, e_{\text{hdg}}^2 + \tfrac{1}{2} w_s\, e_{\text{spd}}^2 + \tfrac{1}{2} w_a\, a^2 + \tfrac{1}{2} w_\delta\, \delta^2
$$

$$
\ell_x = w_l\, e_{\text{lat}} \frac{\partial e_{\text{lat}}}{\partial x} + w_h\, e_{\text{hdg}} \frac{\partial e_{\text{hdg}}}{\partial x} + w_s\, e_{\text{spd}} \frac{\partial e_{\text{spd}}}{\partial x}
$$

$$
\ell_u = \begin{bmatrix} w_a\, a \\ w_\delta\, \delta \end{bmatrix}
$$

$$
\ell_{xx} = w_l \left(\frac{\partial e_{\text{lat}}}{\partial x}\right)^\top \frac{\partial e_{\text{lat}}}{\partial x} + w_h \left(\frac{\partial e_{\text{hdg}}}{\partial x}\right)^\top \frac{\partial e_{\text{hdg}}}{\partial x} + w_s \left(\frac{\partial e_{\text{spd}}}{\partial x}\right)^\top \frac{\partial e_{\text{spd}}}{\partial x}
$$

$$
\ell_{uu} = \operatorname{diag}(w_a,\; w_\delta), \quad \ell_{ux} = 0
$$

即 $\ell_{xx}$ 为三个秩1矩阵之和（每个是外积加权）。

**Terminal cost：** 增加纵向偏差项：

$$
e_{\text{lon}} = \cos h \cdot (p_x - o_x) + \sin h \cdot (p_y - o_y) - s_{\text{target}}, \quad
\frac{\partial e_{\text{lon}}}{\partial x} = \begin{bmatrix} \cos h & \sin h & 0 & 0 \end{bmatrix}
$$

$$
\ell_x = w_{l}^t e_{\text{lon}} \frac{\partial e_{\text{lon}}}{\partial x} + w_{\text{lat}}^t e_{\text{lat}} \frac{\partial e_{\text{lat}}}{\partial x} + w_{h}^t e_{\text{hdg}} \frac{\partial e_{\text{hdg}}}{\partial x} + w_{s}^t e_{\text{spd}} \frac{\partial e_{\text{spd}}}{\partial x}
$$

$$
\ell_{xx} = w_{l}^t \left(\frac{\partial e_{\text{lon}}}{\partial x}\right)^\top \frac{\partial e_{\text{lon}}}{\partial x} + w_{\text{lat}}^t \left(\frac{\partial e_{\text{lat}}}{\partial x}\right)^\top \frac{\partial e_{\text{lat}}}{\partial x} + w_{h}^t \left(\frac{\partial e_{\text{hdg}}}{\partial x}\right)^\top \frac{\partial e_{\text{hdg}}}{\partial x} + w_{s}^t \left(\frac{\partial e_{\text{spd}}}{\partial x}\right)^\top \frac{\partial e_{\text{spd}}}{\partial x}
$$

### 5.4 GuidanceTrackingCost

**Stage cost（非 terminal_mode）：**

$$
\ell = \tfrac{1}{2} w_p (\Delta x^2 + \Delta y^2) + \tfrac{1}{2} w_h\, e_{\text{hdg}}^2 + \tfrac{1}{2} w_s\, e_{\text{spd}}^2
$$

其中 $\Delta x = p_x - r_x$，$\Delta y = p_y - r_y$，$e_{\text{hdg}} = \text{NormalizeAngle}(\theta - r_{\text{yaw}})$，$e_{\text{spd}} = v - r_{\text{speed}}$。

$$
\ell_x = \begin{bmatrix} w_p \Delta x \\ w_p \Delta y \\ w_h e_{\text{hdg}} \\ w_s e_{\text{spd}} \end{bmatrix}, \quad
\ell_u = \mathbf{0}, \quad
\ell_{xx} = \operatorname{diag}(w_p,\; w_p,\; w_h,\; w_s), \quad
\ell_{uu} = 0, \quad
\ell_{ux} = 0
$$

**Terminal cost（terminal_longitudinal_only 模式）：**

$$
e_{\text{lon}} = \cos h \cdot (p_x - r_x) + \sin h \cdot (p_y - r_y), \quad
\frac{\partial e_{\text{lon}}}{\partial x} = \begin{bmatrix} \cos h & \sin h & 0 & 0 \end{bmatrix}
$$

$$
\ell_x = w_p\, e_{\text{lon}} \frac{\partial e_{\text{lon}}}{\partial x}, \quad
\ell_{xx} = w_p \left(\frac{\partial e_{\text{lon}}}{\partial x}\right)^\top \frac{\partial e_{\text{lon}}}{\partial x}
$$

### 5.5 ControlRateCost

$$
\ell = \tfrac{1}{2} w_j (a - a_{\text{prev}})^2 + \tfrac{1}{2} w_{sr} (\delta - \delta_{\text{prev}})^2
$$

$$
\ell_x = 0, \quad
\ell_u = \begin{bmatrix} w_j(a - a_{\text{prev}}) \\ w_{sr}(\delta - \delta_{\text{prev}}) \end{bmatrix}, \quad
\ell_{xx} = 0, \quad
\ell_{uu} = \operatorname{diag}(w_j,\; w_{sr}), \quad
\ell_{ux} = 0
$$

### 5.6 ControlBoxConstraint — Jacobian

$$
c = \begin{bmatrix} u_{\min} - u \\ u - u_{\max} \end{bmatrix} \in \mathbb{R}^{2m}
$$

$$
\frac{\partial c}{\partial x} = 0_{2m \times n}, \qquad
\frac{\partial c}{\partial u} = \begin{bmatrix} -I_m \\ I_m \end{bmatrix}
$$

### 5.7 SpeedLimitConstraint — Jacobian

$$
c = \begin{bmatrix} v_{\min} - v \\ v - v_{\max} \end{bmatrix} \in \mathbb{R}^{2}
$$

$$
\frac{\partial c}{\partial x} = \begin{bmatrix} 0 & 0 & 0 & -1 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \qquad
\frac{\partial c}{\partial u} = 0_{2 \times 2}
$$

### 5.8 TerminalGoalConstraint — Jacobian

$$
c = x - x_{\text{target}} \in \mathbb{R}^{n}
$$

$$
\frac{\partial c}{\partial x} = I_n
$$

$\partial c / \partial u$ 不适用（终端无控制）。

### 5.9 RoadBoundaryConstraint — Jacobian

$$
e_{\text{lat}} = -\sin h \cdot (p_x - o_x) + \cos h \cdot (p_y - o_y)
$$

$$
c = \begin{bmatrix} d_{\text{lb}} - e_{\text{lat}} \\ e_{\text{lat}} - d_{\text{ub}} \end{bmatrix}, \qquad
\frac{\partial e_{\text{lat}}}{\partial x} = \begin{bmatrix} -\sin h & \cos h & 0 & 0 \end{bmatrix}
$$

$$
\frac{\partial c}{\partial x} = \begin{bmatrix} \sin h & -\cos h & 0 & 0 \\ -\sin h & \cos h & 0 & 0 \end{bmatrix}, \qquad
\frac{\partial c}{\partial u} = 0_{2 \times 2}
$$

### 5.10 CircularObstacleConstraint — Jacobian

$$
\Delta x = p_x - c_x, \quad \Delta y = p_y - c_y, \quad c = r^2 - (\Delta x^2 + \Delta y^2)
$$

$$
\frac{\partial c}{\partial x} = \begin{bmatrix} -2\Delta x & -2\Delta y & 0 & 0 \end{bmatrix}, \qquad
\frac{\partial c}{\partial u} = 0_{1 \times 2}
$$

### 5.11 MultiCircleRoadBoundaryConstraint — Jacobian

车身覆盖圆 $j$ 的体坐标偏移 $(b_{x_j}, 0)$（$b_y = 0$，圆心在纵轴上）。

**刚体变换及其 Jacobian（公共子式）：**

$$
\mathbf{p}_j = \begin{bmatrix} p_x + \cos\theta \cdot b_{x_j} \\ p_y + \sin\theta \cdot b_{x_j} \end{bmatrix}
$$

$$
\frac{\partial p_{j,x}}{\partial x} = \begin{bmatrix} 1 & 0 & -\sin\theta \cdot b_{x_j} & 0 \end{bmatrix}, \quad
\frac{\partial p_{j,y}}{\partial x} = \begin{bmatrix} 0 & 1 & \cos\theta \cdot b_{x_j} & 0 \end{bmatrix}
$$

横向偏差（参考线坐标系下）：

$$
\text{lat}_j = -\sin h \cdot (p_{j,x} - o_x) + \cos h \cdot (p_{j,y} - o_y)
$$

$$
\frac{\partial \text{lat}_j}{\partial x} = -\sin h \cdot \frac{\partial p_{j,x}}{\partial x} + \cos h \cdot \frac{\partial p_{j,y}}{\partial x} = \begin{bmatrix} -\sin h & \cos h & b_{x_j} \cos(\theta - h) & 0 \end{bmatrix}
$$

约束输出：

$$
c_{2j} = (d_{\text{lb}} + r_j) - \text{lat}_j, \qquad c_{2j+1} = \text{lat}_j - (d_{\text{ub}} - r_j)
$$

$$
\frac{\partial c_{2j}}{\partial x} = -\frac{\partial \text{lat}_j}{\partial x} = \begin{bmatrix} \sin h & -\cos h & -b_{x_j}\cos(\theta-h) & 0 \end{bmatrix}
$$

$$
\frac{\partial c_{2j+1}}{\partial x} = \frac{\partial \text{lat}_j}{\partial x}, \qquad \frac{\partial c}{\partial u} = 0
$$

### 5.12 MultiCircleVehicleObstacleConstraint — Jacobian

$$
\mathbf{p}_j = \begin{bmatrix} p_x + \cos\theta \cdot b_{x_j} \\ p_y + \sin\theta \cdot b_{x_j} \end{bmatrix}, \quad
\Delta x_j = p_{j,x} - o_{cx}, \quad \Delta y_j = p_{j,y} - o_{cy}
$$

$$
c_j = (r_{\text{obs}} + r_j)^2 - (\Delta x_j^2 + \Delta y_j^2)
$$

$$
\frac{\partial c_j}{\partial x} = \begin{bmatrix} -2\Delta x_j & -2\Delta y_j & 2\, b_{x_j}(\Delta x_j \sin\theta - \Delta y_j \cos\theta) & 0 \end{bmatrix}, \qquad \frac{\partial c_j}{\partial u} = 0
$$

### 5.13 LSEPolygonObstacleConstraint — Jacobian

**LSE 函数对点 $\mathbf{p}$ 的梯度：**

$$
\text{LSE}(\mathbf{p}) = h_{\max} + \frac{1}{\alpha} \ln \sum_{i} \exp\!\bigl(\alpha (h_i - h_{\max})\bigr)
$$

其中 $h_i = \mathbf{n}_i^\top \mathbf{p} - b_i$，$h_{\max} = \max_i h_i$。

softmax 权重：

$$
w_i = \frac{\exp\!\bigl(\alpha (h_i - h_{\max})\bigr)}{\sum_{k} \exp\!\bigl(\alpha (h_k - h_{\max})\bigr)}
$$

$$
\frac{\partial\, \text{LSE}}{\partial \mathbf{p}} = \sum_i w_i \, \mathbf{n}_i \quad \text{（法向量的加权平均）}
$$

约束输出：

$$
c_j = r_j - \text{LSE}(\mathbf{p}_j)
$$

$$
\frac{\partial c_j}{\partial \mathbf{p}} = -\frac{\partial\, \text{LSE}}{\partial \mathbf{p}} = -\sum_i w_i \, \mathbf{n}_i
$$

$$
\frac{\partial c_j}{\partial x} = \left(\frac{\partial c_j}{\partial \mathbf{p}}\right)^\top \frac{\partial \mathbf{p}_j}{\partial x} \quad (1 \times 4)
$$

其中：

$$
\frac{\partial \mathbf{p}_j}{\partial x} = \begin{bmatrix} 1 & 0 & -\sin\theta \cdot b_{x_j} & 0 \\ 0 & 1 & \cos\theta \cdot b_{x_j} & 0 \end{bmatrix}, \qquad \frac{\partial c_j}{\partial u} = 0
$$

### 5.14 增广拉格朗日（AL）罚项展开公式

给定约束 $c(x,u) \in \mathbb{R}^p$，其 Jacobian 为 $J_x = \frac{\partial c}{\partial x} \in \mathbb{R}^{p \times n}$，$J_u = \frac{\partial c}{\partial u} \in \mathbb{R}^{p \times m}$。

**等式约束 AL 项：**

$$
\varphi = \sum_j \left[\lambda_j c_j + \frac{\mu_j}{2} c_j^2\right]
$$

$$
\frac{\partial \varphi}{\partial x} = J_x^\top (\lambda + \mu \odot c), \qquad
\frac{\partial \varphi}{\partial u} = J_u^\top (\lambda + \mu \odot c)
$$

$$
\frac{\partial^2 \varphi}{\partial x^2} \approx J_x^\top \operatorname{diag}(\mu)\, J_x, \qquad
\frac{\partial^2 \varphi}{\partial u^2} \approx J_u^\top \operatorname{diag}(\mu)\, J_u, \qquad
\frac{\partial^2 \varphi}{\partial u \partial x} \approx J_u^\top \operatorname{diag}(\mu)\, J_x
$$

> 以上 Hessian 使用 Gauss-Newton 近似，忽略二阶约束项 $\sum_j (\lambda_j + \mu_j c_j) \nabla^2 c_j$。

**不等式约束 AL 项：**

$$
\varphi = \sum_j \frac{\bigl[\max(0,\;\lambda_j + \mu_j c_j)\bigr]^2 - \lambda_j^2}{2\mu_j}
$$

定义活跃指示 $\mathcal{I}_j = \mathbb{1}[\lambda_j + \mu_j c_j > 0]$，投影值 $\hat{p}_j = \max(0,\;\lambda_j + \mu_j c_j)$。

$$
\frac{\partial \varphi}{\partial x} = J_x^\top (\mathcal{I} \odot \hat{p}), \qquad
\frac{\partial \varphi}{\partial u} = J_u^\top (\mathcal{I} \odot \hat{p})
$$

$$
\frac{\partial^2 \varphi}{\partial x^2} \approx J_x^\top \operatorname{diag}(\mathcal{I} \odot \mu)\, J_x, \qquad
\frac{\partial^2 \varphi}{\partial u^2} \approx J_u^\top \operatorname{diag}(\mathcal{I} \odot \mu)\, J_u, \qquad
\frac{\partial^2 \varphi}{\partial u \partial x} \approx J_u^\top \operatorname{diag}(\mathcal{I} \odot \mu)\, J_x
$$

**总的 AL 代价展开：**

$$
\boxed{\text{总展开} = \text{基础代价展开} + \sum_i \text{第}\,i\,\text{个约束 AL 项展开}}
$$

---

## 六、验证方法

1. **UI 对比**：在 Derivative Mode 下拉框中切换 `Finite Difference` ↔ `Analytical`，观察：
   - 规划耗时变化（Solver 面板）
   - 轨迹是否一致（场景视图叠加对比）
   - 收敛行为是否一致（违约度、代价历史）

2. **数值一致性**：在相同场景下，两种模式应产生相同或极接近的轨迹（解析模式精度更高，微小差异源于差分截断误差）。

3. **回退安全性**：当使用 `PolygonCollisionConstraint`（`kPolygonExact` 模式）时，即使选择 `Analytical`，该约束会自动回退到差分，不影响求解正确性。
