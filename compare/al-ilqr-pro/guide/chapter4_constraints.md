# Chapter 4: 约束建模

> **对应代码**：Phase 5  
> **目标**：理解如何用统一接口描述各种约束，并封装为"带约束的最优控制问题"

---

## 4.1 为什么需要约束

### 基础概念

在 Chapter 3 中，iLQR 能求解无约束最优控制问题。但现实中的系统总存在约束：

- **物理约束**：方向盘有最大转角、加速度有极限
- **安全约束**：不能撞到障碍物、不能开出车道
- **目标约束**：必须到达指定终点

没有约束的轨迹优化可能产生不切实际的解（例如加速度无穷大、穿过障碍物）。

### 约束的数学形式

约束分为两类：

$$
\text{等式约束（Equality）：} \quad c(x, u) = 0
$$

$$
\text{不等式约束（Inequality）：} \quad c(x, u) \leq 0
$$

其中 $c: \mathbb{R}^n \times \mathbb{R}^m \to \mathbb{R}^p$ 是约束函数，$p$ 是约束维度（一个约束函数可以有多个输出维度）。

### 违约度（Violation）

衡量约束被违反的程度：

$$
\text{等式约束违约度：} \quad \|c(x, u)\|_\infty = \max_i |c_i|
$$

$$
\text{不等式约束违约度：} \quad \max(0, \max_i c_i)
$$

对应代码（`src/constraints/constraint_function.cpp`）：

```cpp
double MaxViolation(const ConstraintFunction& constraint, const Vector& values) {
  if (constraint.Type() == ConstraintType::kEquality) {
    return values.cwiseAbs().maxCoeff();     // max |c_i|
  }
  double max_violation = 0.0;
  for (int i = 0; i < values.size(); ++i) {
    max_violation = std::max(max_violation, values(i));
  }
  return std::max(0.0, max_violation);       // max(0, max c_i)
}
```

---

## 4.2 约束接口设计

### 统一的约束接口

本项目通过一个纯虚基类统一所有约束：

```
┌─────────────────────────────────────────┐
│          ConstraintFunction             │
│           (抽象基类)                     │
│                                         │
│  Type()      → kEquality / kInequality  │
│  StateDim()  → 状态维度                  │
│  ControlDim()→ 控制维度                  │
│  OutputDim() → 约束输出维度              │
│  Name()      → 约束名称                  │
│  Evaluate(x, u) → 约束值向量            │
└───────────┬─────────────────────────────┘
            │ 继承
     ┌──────┼──────────┬─────────────┐
     │      │          │             │
     ▼      ▼          ▼             ▼
 Control  Terminal   Road       Obstacle
  Box      Goal    Boundary    Constraint
```

```cpp
// include/constraints/constraint_function.hpp
class ConstraintFunction {
 public:
  virtual ConstraintType Type() const = 0;        // 等式 or 不等式
  virtual int StateDim() const = 0;               // 需要的状态维度
  virtual int ControlDim() const = 0;             // 需要的控制维度
  virtual int OutputDim() const = 0;              // 约束值的维度 p
  virtual std::string Name() const = 0;           // 用于调试
  virtual Vector Evaluate(const Vector& state,    // 核心：计算约束值
                          const Vector& control) const = 0;
};
```

---

## 4.3 具体约束实现

### 4.3.1 控制量上下界约束（ControlBoxConstraint）

限制控制量在 $[u_{\min}, u_{\max}]$ 范围内。

```
   约束含义:   u_min ≤ u ≤ u_max

   转化为标准形式 c(x,u) ≤ 0:
     c₁ = u_min - u ≤ 0     (下界)
     c₂ = u - u_max ≤ 0     (上界)
```

对于 $m$ 维控制量，约束维度 $p = 2m$。

对应代码（`src/constraints/control_box_constraint.cpp`）：

```cpp
Vector ControlBoxConstraint::Evaluate(const Vector& state, const Vector& control) const {
  Vector values(OutputDim());                    // 2m 维输出
  values.head(ControlDim()) = lower_bound_ - control;   // u_min - u ≤ 0
  values.tail(ControlDim()) = control - upper_bound_;   // u - u_max ≤ 0
  return values;
}
```

示例：对自行车模型，$u = [a, \delta]^T$，约束 $a \in [-3, 3]$，$\delta \in [-0.5, 0.5]$：

```
输出维度: 4
c = [(-3) - a,  (-0.5) - δ,  a - 3,  δ - 0.5]ᵀ
     ────────────────────    ──────────────────
      下界约束 (2维)           上界约束 (2维)
```

### 4.3.2 终端目标约束（TerminalGoalConstraint）

要求终端状态精确等于目标状态——这是**等式约束**。

$$
c(x_N) = x_N - x_{\text{target}} = 0
$$

对应代码（`src/constraints/terminal_goal_constraint.cpp`）：

```cpp
Vector TerminalGoalConstraint::Evaluate(const Vector& state, const Vector& control) const {
  return state - target_state_;   // x_N - x_target = 0
}
```

注意：终端约束不依赖控制量（`ControlDim() = 0`）。

---

## 4.4 带约束的最优控制问题

### ConstrainedOptimalControlProblem

在 `OptimalControlProblem` 的基础上，添加约束管理功能：

```
┌──────────────────────────────────────────────────────────┐
│       ConstrainedOptimalControlProblem                    │
│                                                          │
│  problem_ ──→ OptimalControlProblem (动力学+代价)         │
│                                                          │
│  stage_constraints_:                                     │
│    k=0: [ControlBox, ObstacleA, ObstacleB, ...]          │
│    k=1: [ControlBox, ObstacleA, ObstacleB, ...]          │
│    ...                                                   │
│    k=N-1: [ControlBox, ObstacleA, ObstacleB, ...]        │
│                                                          │
│  terminal_constraints_:                                  │
│    [TerminalGoal]                                        │
│                                                          │
│  AddStageConstraint(k, constraint)                       │
│  AddTerminalConstraint(constraint)                       │
│  EvaluateTrajectory(traj) → 所有约束的评估               │
│  MaxViolation(traj) → 最大违约度                         │
└──────────────────────────────────────────────────────────┘
```

### 约束添加方式

阶段约束按 knot point 独立管理。**每个 knot point 可以有不同的约束集合**（虽然通常相同）：

```cpp
// 为每个时间步添加控制约束
for (int k = 0; k < horizon; ++k) {
  constrained_problem.AddStageConstraint(k, control_box);
}

// 只在终端添加目标约束
constrained_problem.AddTerminalConstraint(terminal_goal);
```

对应代码（`src/problems/constrained_optimal_control_problem.cpp`）：

```cpp
void ConstrainedOptimalControlProblem::AddStageConstraint(int k, ConstraintPtr constraint) {
  if (k < 0 || k >= Horizon()) {
    throw std::out_of_range("Stage constraint index is out of range.");
  }
  stage_constraints_[k].push_back(std::move(constraint));  // 添加到第 k 步的约束列表
}
```

### 约束评估流程

```
                  EvaluateTrajectory 流程

  ┌─────────────────────────────────────────────────┐
  │ for k = 0 to N-1:                               │
  │   for each constraint in stage_constraints_[k]: │
  │     values = constraint.Evaluate(x_k, u_k)      │
  │     violation = MaxViolation(constraint, values) │
  │     记录 {name, type, k, values, violation}      │
  │                                                  │
  │ for each constraint in terminal_constraints_:    │
  │   values = constraint.Evaluate(x_N, 空控制)      │
  │   记录 {name, type, N, values, violation}        │
  └─────────────────────────────────────────────────┘
```

对应代码：

```cpp
std::vector<ConstraintEvaluation> ConstrainedOptimalControlProblem::EvaluateTrajectory(
    const Trajectory& trajectory) const {
  std::vector<ConstraintEvaluation> evaluations;

  // 评估所有阶段约束
  for (int k = 0; k < Horizon(); ++k) {
    for (const auto& constraint : stage_constraints_[k]) {
      const Vector values = constraint->Evaluate(trajectory.State(k), trajectory.Control(k));
      evaluations.push_back({constraint->Name(), constraint->Type(), k, values,
                             my_al_ilqr::MaxViolation(*constraint, values)});
    }
  }

  // 评估终端约束
  const Vector empty_control = Vector::Zero(0);
  for (const auto& constraint : terminal_constraints_) {
    const Vector values = constraint->Evaluate(trajectory.State(Horizon()), empty_control);
    evaluations.push_back({constraint->Name(), constraint->Type(), Horizon(), values,
                           my_al_ilqr::MaxViolation(*constraint, values)});
  }
  return evaluations;
}
```

---

## 4.5 约束的维度计数

理解约束维度对后续的增广拉格朗日方法至关重要：

```
   例: 自行车模型, horizon=50, 2个障碍物, 车体2个圆

   每个 knot point 的约束:
   ┌────────────────────────────────────────┐
   │ ControlBoxConstraint:                  │
   │   OutputDim = 2 × 2 = 4  (a和δ的上下界) │
   │                                        │
   │ ObstacleConstraint (障碍物1):           │
   │   OutputDim = 2  (2个圆 × 1个障碍物)    │
   │                                        │
   │ ObstacleConstraint (障碍物2):           │
   │   OutputDim = 2  (2个圆 × 1个障碍物)    │
   │                                        │
   │ 每个 knot: 4 + 2 + 2 = 8 个约束值       │
   └────────────────────────────────────────┘

   总约束数 (阶段): 8 × 50 = 400
   终端约束: TerminalGoal → OutputDim = 4
   
   总计: 404 个约束值
   每个约束值都需要一个 λ（拉格朗日乘子）和一个 μ（惩罚参数）
```

---

## 4.6 本章小结

```
┌───────────────────────────────────────────────────────┐
│                 本章模块关系图                          │
│                                                       │
│  ConstraintFunction (接口)                            │
│  ├── ControlBoxConstraint (不等式, c = u_lb - u ≤ 0) │
│  ├── TerminalGoalConstraint (等式, c = x - x_tgt = 0)│
│  ├── RoadBoundaryConstraint (后续章节)                │
│  ├── SpeedLimitConstraint (后续章节)                  │
│  └── ObstacleConstraint (后续章节)                    │
│           │                                           │
│           ▼                                           │
│  ConstrainedOptimalControlProblem                     │
│  ├── OptimalControlProblem (基础问题)                 │
│  ├── stage_constraints_[k] (每步的约束列表)            │
│  ├── terminal_constraints_ (终端约束列表)              │
│  └── MaxViolation() (评估违约度)                      │
└───────────────────────────────────────────────────────┘
```

**核心收获**：
- 所有约束统一转化为 $c(x,u) \leq 0$ 或 $c(x,u) = 0$ 的标准形式
- 约束按 knot point 独立管理，每步可以有不同约束
- 此时能**建模**约束，但还不能**求解**带约束问题——这需要 Chapter 5 的增广拉格朗日方法
