# Chapter 5: 增广拉格朗日方法

> **对应代码**：Phase 6  
> **目标**：理解如何将约束问题转化为无约束问题，使 iLQR 能处理约束

---

## 5.1 约束处理的核心挑战

### 基础概念

我们在 Chapter 3 实现了无约束 iLQR，在 Chapter 4 定义了各种约束。问题是：**iLQR 只能求解无约束问题，如何让它处理约束？**

核心思路：**将约束"转化"为代价函数的一部分**，使得违反约束会产生高代价。这样，原来的带约束问题就变成了一个无约束问题，可以直接用 iLQR 求解。

### 三种主要方法对比

```
方法1: 惩罚法 (Penalty Method)
  J_new = J + μ/2 · ||c(x,u)||²
  问题: μ 太小→约束不满足; μ 太大→数值病态

方法2: 拉格朗日对偶法 (Lagrangian)
  L = J + λᵀ c(x,u)
  问题: 需要鞍点条件, 不适合直接优化

方法3: 增广拉格朗日法 (Augmented Lagrangian) ← 本项目采用
  L_A = J + λᵀ c(x,u) + μ/2 · ||c(x,u)||²
  优点: 结合了前两者的优势!
```

---

## 5.2 增广拉格朗日方法原理

### 5.2.1 等式约束

对于等式约束 $c(x,u) = 0$，增广拉格朗日函数为：

$$
\mathcal{L}_A = J + \lambda^T c + \frac{\mu}{2} \|c\|^2
$$

展开到每个约束分量 $c_i$：

$$
\mathcal{L}_{A,i} = \lambda_i \cdot c_i + \frac{\mu_i}{2} \cdot c_i^2
$$

- $\lambda_i$：第 $i$ 个约束的**拉格朗日乘子（对偶变量）**，表示约束的"价格"
- $\mu_i$：第 $i$ 个约束的**惩罚参数**，控制惩罚强度

### 5.2.2 不等式约束

对于不等式约束 $c(x,u) \leq 0$，需要特殊处理——只在约束被违反（或接近活跃）时施加惩罚：

$$
\mathcal{L}_{A,i} = \frac{1}{2\mu_i}\left[\max(0, \lambda_i + \mu_i c_i)^2 - \lambda_i^2\right]
$$

这个公式的直觉：

```
情况 1: 约束远未活跃 (c_i << 0, λ_i = 0)
  → max(0, 0 + μ·c_i) = 0
  → L_A = (0 - 0) / 2μ = 0  ← 不施加任何惩罚!

情况 2: 约束刚好活跃 (c_i ≈ 0, λ_i > 0)
  → max(0, λ_i + μ·c_i) ≈ λ_i
  → L_A ≈ 0  ← 小惩罚

情况 3: 约束被违反 (c_i > 0)
  → max(0, λ_i + μ·c_i) = λ_i + μ·c_i > 0
  → L_A > 0  ← 大惩罚!
```

### 5.2.3 对偶变量更新

在每次外层迭代后，更新对偶变量：

$$
\text{等式约束：} \quad \lambda_i^+ = \lambda_i + \mu_i \cdot c_i
$$

$$
\text{不等式约束：} \quad \lambda_i^+ = \max(0, \lambda_i + \mu_i \cdot c_i)
$$

不等式约束的 $\max(0, \cdot)$ 确保 $\lambda_i \geq 0$（非负性，对应 KKT 条件）。

### 5.2.4 惩罚参数更新

如果约束违反还较大，增大惩罚：

$$
\mu_i^+ = \phi \cdot \mu_i \quad (\phi > 1, \text{如 } \phi = 10)
$$

---

## 5.3 代码实现

### 5.3.1 数据结构

每个约束在 AL 中需要额外的数据：

```cpp
// include/al/augmented_lagrangian_cost.hpp
struct AugmentedLagrangianConstraintData {
  ConstraintPtr constraint;      // 指向原始约束
  Vector lambda;                 // 对偶变量 λ (p×1)
  Vector mu;                     // 惩罚参数 μ (p×1)
  double penalty_scaling = 10.0; // 缩放因子 φ
};
```

初始化时，$\lambda = 0$, $\mu$ 设为初始惩罚值：

```cpp
AugmentedLagrangianConstraintData MakeConstraintData(const ConstraintPtr& constraint,
                                                     double initial_penalty,
                                                     double penalty_scaling) {
  return {
    constraint,
    Vector::Zero(constraint->OutputDim()),                   // λ 初始为 0
    Vector::Constant(constraint->OutputDim(), initial_penalty), // μ 初始为 initial_penalty
    penalty_scaling                                          // φ
  };
}
```

### 5.3.2 增广拉格朗日代价计算

`AugmentedLagrangianKnotCost` 包装了原始代价函数，在其基础上添加 AL 罚项：

```
   原始代价: ℓ(x, u)
       │
       ▼
   AL 代价: ℓ(x, u) + Σ AL_term_i(x, u)
                       ↑
                 对每个约束 i，添加 AL 罚项
```

核心计算代码（`src/al/augmented_lagrangian_cost.cpp`）：

```cpp
double AugmentedLagrangianKnotCost::ConstraintAugmentedTerm(
    const AugmentedLagrangianConstraintData& data,
    const Vector& state,
    const Vector& control) const {

  const Vector values = data.constraint->Evaluate(state, control);  // 计算约束值 c
  double total = 0.0;

  for (int i = 0; i < values.size(); ++i) {
    if (data.constraint->Type() == ConstraintType::kEquality) {
      // 等式约束: λᵢcᵢ + μᵢ/2 · cᵢ²
      total += data.lambda(i) * values(i)
             + 0.5 * data.mu(i) * values(i) * values(i);
    } else {
      // 不等式约束: [max(0, λᵢ + μᵢcᵢ)² - λᵢ²] / (2μᵢ)
      const double projected = std::max(0.0, data.lambda(i) + data.mu(i) * values(i));
      total += (projected * projected - data.lambda(i) * data.lambda(i))
             / (2.0 * data.mu(i));
    }
  }
  return total;
}
```

将 AL 项加到原始代价上：

```cpp
double AugmentedLagrangianKnotCost::StageCost(const Vector& state, const Vector& control) const {
  double cost = base_cost_->StageCost(state, control);   // 原始代价
  for (const auto& data : constraints_) {
    cost += ConstraintAugmentedTerm(data, state, control);  // + AL 罚项
  }
  return cost;
}
```

### 5.3.3 对偶变量更新

```cpp
void AugmentedLagrangianKnotCost::UpdateDuals(const Vector& state, const Vector& control) {
  for (auto& data : constraints_) {
    const Vector values = data.constraint->Evaluate(state, control);

    if (data.constraint->Type() == ConstraintType::kEquality) {
      // λ⁺ = λ + μ ⊙ c    (逐元素乘)
      data.lambda += data.mu.cwiseProduct(values);
    } else {
      // λ⁺ = max(0, λ + μ ⊙ c)
      data.lambda = (data.lambda + data.mu.cwiseProduct(values)).cwiseMax(0.0);
    }
  }
}
```

### 5.3.4 惩罚参数更新

```cpp
void AugmentedLagrangianKnotCost::UpdatePenalties() {
  for (auto& data : constraints_) {
    data.mu *= data.penalty_scaling;   // μ⁺ = φ · μ
  }
}
```

---

## 5.4 AugmentedLagrangianProblem：问题包装

### 结构

`AugmentedLagrangianProblem` 将带约束问题转化为无约束子问题：

```
┌─────────────────────────────────────────────────────────────┐
│      AugmentedLagrangianProblem                              │
│                                                             │
│  输入: ConstrainedOptimalControlProblem                      │
│  ├── OptimalControlProblem (动力学+代价)                     │
│  ├── stage_constraints_[k]                                  │
│  └── terminal_constraints_                                  │
│                                                             │
│  转化为:                                                     │
│  ├── stage_costs_[k]: AugmentedLagrangianKnotCost           │
│  │     = 原始代价 + AL罚项(该步所有约束)                      │
│  ├── terminal_cost_: AugmentedLagrangianKnotCost            │
│  │     = 原始终端代价 + AL罚项(终端约束)                      │
│  └── subproblem_: OptimalControlProblem (无约束!)             │
│        = 动力学 + 包装后的代价                                │
│                                                             │
│  API:                                                       │
│  ├── UnconstrainedSubproblem() → 交给 iLQR 求解              │
│  ├── UpdateDuals(trajectory) → 更新所有 λ                    │
│  ├── UpdatePenalties() → 更新所有 μ                          │
│  └── MaxViolation(trajectory) → 检查收敛                     │
└─────────────────────────────────────────────────────────────┘
```

### 转化流程

```
原始问题:                       转化后:

  min  Σ ℓ_k(x,u) + ℓ_N(x)      min  Σ [ℓ_k(x,u) + AL_k(x,u)]
  s.t. c_k(x,u) ≤ 0               + [ℓ_N(x) + AL_N(x)]
       c_N(x) = 0
                                    (无约束!)
   ▼                                    ▼
  需要特殊求解器                      直接用 iLQR 求解!
```

对应代码（构造函数中的核心逻辑）：

```cpp
AugmentedLagrangianProblem::AugmentedLagrangianProblem(
    const ConstrainedOptimalControlProblem& constrained_problem,
    double initial_penalty, double penalty_scaling) {
  auto base_problem = constrained_problem.SharedBaseProblem();

  // 对每个 knot point，创建包装代价
  for (int k = 0; k < base_problem->Horizon(); ++k) {
    std::vector<AugmentedLagrangianConstraintData> constraints;
    for (const auto& constraint : constrained_problem.StageConstraints(k)) {
      constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
    }
    auto knot_cost = std::make_shared<AugmentedLagrangianKnotCost>(
        base_problem->SharedStageCostFunction(k), constraints);
    stage_costs_.push_back(knot_cost);
  }

  // 终端约束
  std::vector<AugmentedLagrangianConstraintData> terminal_constraints;
  for (const auto& constraint : constrained_problem.TerminalConstraints()) {
    terminal_constraints.push_back(MakeConstraintData(constraint, initial_penalty, penalty_scaling));
  }
  terminal_cost_ = std::make_shared<AugmentedLagrangianKnotCost>(
      base_problem->SharedTerminalCostFunction(), terminal_constraints);

  // 创建无约束子问题 (动力学不变, 代价被包装)
  subproblem_ = std::make_shared<OptimalControlProblem>(
      base_problem->SharedDynamics(), stage_costs, terminal_cost_, base_problem->TimeStep());
}
```

---

## 5.5 实现路径：数值差分 vs 解析展开

> **重要实现细节**：AL-iLQR 论文中通常将 AL 项显式展开——手推 $\nabla_{x} \mathcal{L}_A$、$\nabla_{u} \mathcal{L}_A$ 等，其中涉及约束的 Jacobian $c_x, c_u$。
>
> **本项目的做法不同**：`AugmentedLagrangianKnotCost` 将 AL 罚项包进 `StageCost()` 返回值中，然后由 `ILQRSolver` 对**整个代价**（基础代价 + AL 项）统一做数值中心差分来获取梯度和 Hessian。
>
> 两种方式**数学等价**，最终优化结果一致。区别在于：
> - 论文方式：需要手推 $c_x, c_u$ 的解析表达式，但计算效率高
> - 本项目方式：无需推导约束 Jacobian，新增约束只要实现 `Evaluate()` 即可，但每次差分需要多次调用约束函数
>
> 关于与论文的完整差异清单，参见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)。

```
论文标准做法:                    本项目做法:
                                
  ℓ_x, ℓ_u (代价梯度)             StageCost(x,u)
  c_x, c_u (约束 Jacobian)          = base_cost + AL 项
  → 手动组合为 AL 梯度              → 整体做数值差分
  → 精确但需要推导                  → 简洁但计算量大
```

---

## 5.6 AL 的迭代过程（预告 Chapter 6）

增广拉格朗日方法是一个双层循环：

```
外层循环 (AL):
┌──────────────────────────────────────────────────┐
│  repeat:                                         │
│    ① 用 iLQR 求解当前的无约束子问题                 │
│       → 得到轨迹 {x_k, u_k}                      │
│                                                  │
│    ② 用当前轨迹更新对偶变量 λ                      │
│       → λ⁺ = λ + μ·c   (等式)                    │
│       → λ⁺ = max(0, λ + μ·c) (不等式)            │
│                                                  │
│    ③ 检查收敛: max_violation < tolerance ?         │
│       → 是 → 结束                                │
│       → 否 → 继续                                │
│                                                  │
│    ④ 更新惩罚参数 μ (可选, 根据进展)               │
│       → μ⁺ = φ · μ                               │
│                                                  │
│    ⑤ 回到 ①                                      │
│  until 收敛 or 达到最大迭代次数                     │
└──────────────────────────────────────────────────┘

内层循环 (iLQR):
  就是 Chapter 3 的 iLQR，对无约束子问题求解
```

---

## 5.7 为什么 AL 能收敛

直觉解释：

1. **初始阶段**：$\lambda = 0$, $\mu$ 较小 → AL 项较弱，iLQR 主要优化原始代价
2. **对偶更新**：$\lambda$ 逐渐"学会"了约束的重要性
   - 被违反严重的约束 → $\lambda$ 增长快 → 对该约束的惩罚加重
   - 已满足的不等式约束 → $\max(0,\cdot)$ 截断 → $\lambda$ 不增长
3. **惩罚增大**：$\mu$ 增大让违约"更贵"，迫使 iLQR 更积极地满足约束
4. **收敛**：当 $\lambda$ 收敛到真实的最优乘子时，$\mu$ 不需要趋于无穷大（这是 AL 优于纯惩罚法的关键）

---

## 5.8 本章小结

```
┌─────────────────────────────────────────────────────────┐
│               增广拉格朗日方法核心思路                      │
│                                                         │
│  带约束问题                                              │
│    min J  s.t. c(x,u) ≤ 0                               │
│                                                         │
│          ┌──────── AL 转化 ────────┐                    │
│          ▼                         │                    │
│  无约束子问题                       │  更新 λ, μ         │
│    min J + λᵀc + μ/2·||c||²        │  检查收敛           │
│          │                         │                    │
│          ▼                         │                    │
│  iLQR 求解                         │                    │
│          │                         │                    │
│          └─────── 轨迹结果 ─────────┘                    │
│                                                         │
│  关键数据结构:                                           │
│  ├── AugmentedLagrangianConstraintData (λ, μ per 约束)  │
│  ├── AugmentedLagrangianKnotCost (base_cost + AL 项)    │
│  └── AugmentedLagrangianProblem (转化+管理)              │
│                                                         │
│  核心公式:                                               │
│  ├── 等式: λc + μ/2·c²                                  │
│  ├── 不等式: [max(0,λ+μc)² - λ²] / (2μ)                │
│  ├── λ 更新: λ⁺ = max(0, λ+μc) 或 λ+μc                 │
│  └── μ 更新: μ⁺ = φ·μ                                   │
└─────────────────────────────────────────────────────────┘
```

**核心收获**：
- 增广拉格朗日法将约束"软化"为代价项
- 不等式约束通过 $\max(0, \cdot)$ 实现"非活跃约束自动消失"
- 通过迭代更新 $\lambda$ 和 $\mu$，逐步逼近可行解
- 下一章将实现完整的 AL-iLQR 外层循环
