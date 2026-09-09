# Chapter 6: 完整 AL-iLQR 求解器

> **对应代码**：Phase 7  
> **目标**：组装完整的双层循环求解器，真正求解带约束的轨迹优化问题

---

## 6.1 整体架构

### 基础概念

AL-iLQR 是一个**双层循环**结构：

- **外层循环（AL）**：管理约束——更新 $\lambda$ 和 $\mu$，检查约束收敛
- **内层循环（iLQR）**：求解当前的无约束子问题——做 expansion、backward pass、forward pass

```
┌──────────────────────────────── 外层循环 (AL) ──────────────────────────────┐
│                                                                            │
│  outer_iter = 0, 1, 2, ...                                                │
│                                                                            │
│  ┌────────────────────── 内层循环 (iLQR) ──────────────────────┐           │
│  │  展开 → Backward Pass → Forward Pass → 收敛?                │           │
│  │  (多次迭代, 直到内层收敛)                                    │           │
│  └──────────────────────────────────────────────────────────────┘           │
│              │                                                             │
│              ▼                                                             │
│  得到当前最优轨迹                                                           │
│              │                                                             │
│  ┌───────────┴──────────────────────────────────────────┐                  │
│  │ 约束收敛? max_violation < tolerance → 结束             │                  │
│  │                                                      │                  │
│  │ 更新对偶: λ⁺ = λ + μc (等式) / max(0,λ+μc) (不等式)  │                  │
│  │                                                      │                  │
│  │ 惩罚增大? 根据进展决定 μ⁺ = φ·μ                       │                  │
│  └──────────────────────────────────────────────────────┘                  │
│              │                                                             │
│              └──→ 回到外层循环顶部                                          │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 6.2 配置参数

```cpp
// include/al/al_ilqr_solver.hpp
struct ALILQROptions {
  ILQROptions inner_options;           // iLQR 内层配置
  int max_outer_iterations = 10;       // 外层最大迭代次数
  double constraint_tolerance = 1e-3;  // 约束收敛容差
  double initial_penalty = 10.0;       // μ 初始值
  double penalty_scaling = 5.0;        // φ (惩罚缩放因子)
  double penalty_update_ratio = 0.25;  // 进展判断阈值
  double max_penalty = 1e8;            // μ 上限
  bool return_best_trajectory = true;  // 返回最佳轨迹
};
```

各参数的作用：

```
参数关系图:

  initial_penalty (μ₀ = 10)
       │
       │  constraint_tolerance (ε = 1e-3)
       │       │
       ▼       ▼
  ┌────────────────────────┐
  │  if violation < ε:     │ → 收敛! 结束
  │  else:                 │
  │    更新 λ              │
  │    if 进展不足:        │ → μ *= penalty_scaling (5)
  │    if μ > max_penalty: │ → 放弃
  └────────────────────────┘

  penalty_update_ratio (0.25): 
    如果 violation > 0.25 × 上次惩罚更新时的 violation
    → 认为进展不足，需要增大 μ
```

---

## 6.3 求解主流程

### 完整代码解读

```cpp
ALILQRResult ALILQRSolver::Solve(const Vector& initial_state,
                                 const std::vector<Vector>& initial_controls) {
  // 1. 创建 AL 问题（将约束转为代价项）
  AugmentedLagrangianProblem al_problem(problem_, options_.initial_penalty,
                                        options_.penalty_scaling);

  // 2. 初始 rollout
  std::vector<Vector> controls = initial_controls;
  Trajectory current_trajectory = problem_.BaseProblem().Rollout(initial_state, controls);

  // 3. 记录初始违约度
  double current_violation = problem_.MaxViolation(current_trajectory);
  double penalty_reference_violation = current_violation; // 上次更新惩罚时的违约度
  double best_violation = current_violation;
  Trajectory best_trajectory = current_trajectory;        // 记录历史最佳

  // 4. 外层循环
  for (int outer_iter = 0; outer_iter < options_.max_outer_iterations; ++outer_iter) {
    // ── 内层: iLQR 求解 ──
    ILQRSolver inner_solver(*al_problem.UnconstrainedSubproblem(), options_.inner_options);
    current_trajectory = inner_solver.Solve(initial_state, controls);
    controls = ExtractControls(current_trajectory);

    // ── 评估 ──
    const double max_violation = problem_.MaxViolation(current_trajectory);
    if (max_violation < best_violation) {
      best_violation = max_violation;
      best_trajectory = current_trajectory;   // 保存最佳
    }

    // ── 收敛检查 ──
    if (max_violation <= options_.constraint_tolerance) {
      converged = true;
      break;                                  // 约束满足, 成功!
    }

    // ── 更新对偶变量 ──
    al_problem.UpdateDuals(current_trajectory);

    // ── 条件性惩罚更新 ──
    const bool insufficient_progress =
        max_violation > penalty_reference_violation * options_.penalty_update_ratio;
    if (insufficient_progress) {
      al_problem.UpdatePenalties();           // μ *= φ
      penalty_reference_violation = max_violation;
      if (al_problem.MaxPenalty() > options_.max_penalty) {
        break;                                // μ 太大, 放弃
      }
    }
  }

  // 5. 返回最佳结果
  return ALILQRResult{best_trajectory, converged, ...};
}
```

---

## 6.4 惩罚更新策略

> **注意**：本节描述的条件性惩罚更新是**本项目自己的工程启发式（heuristic）**，不是 AL-iLQR 论文中的标准公式。标准 AL 每次外层迭代都无条件地 $\mu \leftarrow \phi \cdot \mu$，本项目的做法更保守，可以避免 $\mu$ 增长过快导致子问题数值病态。关于本项目与论文的全部差异点，参见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)。

### 为什么不每次都增大 μ

简单的策略是每次外层迭代都 $\mu \leftarrow \phi \cdot \mu$，但这会导致 $\mu$ 增长过快，使子问题数值条件恶化。

本项目采用**条件性更新**——只在进展不足时增大 $\mu$，通过两个独立判据触发：

**判据 1**：与上次惩罚更新时相比进展不足

$$
\text{violation} > \text{penalty\_reference\_violation} \times \text{penalty\_update\_ratio}
$$

**判据 2**：与上一次外层迭代相比几乎停滞

$$
\text{violation} > \text{previous\_outer\_violation} \times \text{stagnation\_ratio}
$$

> **代码细节**：`stagnation_ratio` 并非直接使用 `penalty_update_ratio`，而是取 `std::max(penalty_update_ratio, 0.95)`。这意味着当 `penalty_update_ratio < 0.95`（典型值 0.25-0.45）时，第二个判据的阈值**实际固定为 0.95**，即只有违约度下降不到 5% 才算停滞。

```
                    惩罚更新决策流程

  计算 max_violation
       │
       ├── 判据1: violation > penalty_ref × ratio ?
       │        │  (penalty_update_ratio, 如 0.25)
       │        │  → 与上次增大惩罚时比, 减少了 75% 以上?
       │        ├── YES → 进展不足
       │        └── NO  → 已有足够进展
       │
       ├── 判据2: violation > prev_violation × max(ratio, 0.95) ?
       │        │  → 注意: 阈值至少 0.95, 而非直接用 ratio
       │        │  → 即: 与上一轮比, 减少不到 5% 算停滞
       │        ├── YES → 停滞
       │        └── NO  → 在改善
       │
       ├── 任一判据为 YES → μ *= φ, 更新 penalty_ref
       │
       └── μ > max_penalty ? → 终止
```

对应代码（`al_ilqr_solver.cpp`）：

```cpp
const double stagnation_ratio = std::max(options_.penalty_update_ratio, 0.95);
const bool insufficient_progress_since_penalty =
    max_violation > penalty_reference_violation * options_.penalty_update_ratio;
const bool insufficient_progress_since_last_outer =
    outer_iter > 0 && max_violation > previous_outer_violation * stagnation_ratio;

if (insufficient_progress_since_penalty || insufficient_progress_since_last_outer) {
  al_problem.UpdatePenalties();
  penalty_reference_violation = max_violation;
}
```

---

## 6.5 最佳轨迹追踪

AL-iLQR 在迭代过程中，违约度不一定单调下降。因此项目记录历史最佳轨迹：

```
   外层迭代:  1      2      3      4      5
   violation: 0.5    0.1    0.3    0.05   0.001
                                   ↑
                      μ 增大后违约度暂时反弹
   
   best_violation 变化:
              0.5    0.1    0.1    0.05   0.001
                                   ↑
                      保持第2次的最佳值
```

```cpp
if (max_violation < best_violation) {
  best_violation = max_violation;
  best_trajectory = current_trajectory;
}

// 最终返回: 选择 best 或 final 中更好的
const bool use_best = options_.return_best_trajectory && best_violation < final_violation;
```

---

## 6.6 求解日志

每次外层迭代记录详细信息，便于调试和分析：

```cpp
struct ALILQROuterIterationLog {
  int outer_iteration = 0;          // 第几次外层迭代
  int inner_iterations = 0;         // 内层 iLQR 用了多少步
  double base_cost = 0.0;           // 不含 AL 项的原始代价
  double augmented_cost = 0.0;      // 含 AL 项的代价
  double max_violation = 0.0;       // 当前最大违约度
  double best_violation_so_far = 0.0; // 历史最佳违约度
  double max_penalty = 0.0;         // 当前最大 μ
  bool penalty_updated = false;     // 本轮是否更新了惩罚
};
```

典型输出示例：

```
Outer  Inner  Base Cost  Violation  Best     Max μ   Penalty Updated
  1      25    14.532    0.8421    0.8421    10.0     yes
  2      18    16.871    0.2105    0.2105    50.0     yes
  3      12    17.234    0.0412    0.0412    250.0    no
  4       8    17.189    0.0083    0.0083    250.0    no
  5       6    17.195    0.0006    0.0006    250.0    no  ← 收敛!
```

---

## 6.7 完整算法流程图

```
┌──────────────────────────────────────────────────────────────┐
│                    AL-iLQR 完整算法                           │
│                                                              │
│  输入: 带约束OCP, 初始状态 x₀, 初始控制 {u₀,...,u_{N-1}}     │
│                                                              │
│  Step 0: 创建 AL 问题 (约束→代价)                             │
│          λ = 0, μ = μ₀                                       │
│                                                              │
│  Step 1: 初始 Rollout                                        │
│                                                              │
│  for outer = 1, 2, ..., max_outer:                           │
│  │                                                           │
│  │  Step 2: iLQR 求解无约束子问题                              │
│  │  ┌─────────────────────────────────────┐                  │
│  │  │ for inner = 1, 2, ..., max_inner:   │                  │
│  │  │   展开 (A_k, B_k, ℓ_x, ℓ_u, ...)   │                  │
│  │  │   Backward Pass (K_k, d_k)          │                  │
│  │  │   Forward Pass (线搜索)              │                  │
│  │  │   收敛? → break                     │                  │
│  │  └─────────────────────────────────────┘                  │
│  │                                                           │
│  │  Step 3: 检查约束收敛                                      │
│  │    violation < tolerance → 成功, 返回!                     │
│  │                                                           │
│  │  Step 4: 更新对偶 λ                                       │
│  │    等式: λ⁺ = λ + μc                                      │
│  │    不等式: λ⁺ = max(0, λ+μc)                              │
│  │                                                           │
│  │  Step 5: 条件更新惩罚 μ                                    │
│  │    进展不足? → μ⁺ = φμ                                     │
│  │    μ > max_penalty? → 放弃                                │
│  │                                                           │
│  └─→ 回到 Step 2                                             │
│                                                              │
│  输出: ALILQRResult {轨迹, 是否收敛, 违约度, 日志}             │
└──────────────────────────────────────────────────────────────┘
```

---

## 6.8 本章小结

```
┌──────────────────────────────────────────────────────┐
│              AL-iLQR 完整系统                         │
│                                                      │
│  ConstrainedOCP                                      │
│    │  (动力学 + 代价 + 约束)                          │
│    │                                                 │
│    ▼                                                 │
│  AugmentedLagrangianProblem                          │
│    │  (约束 → AL 代价项)                              │
│    │                                                 │
│    ▼                                                 │
│  ALILQRSolver                                        │
│    ├─ 外层: 管理 λ, μ, 收敛检查                       │
│    └─ 内层: ILQRSolver (展开+backward+forward)        │
│                                                      │
│  特色:                                               │
│  ├── 条件性惩罚更新 (避免 μ 增长过快)                  │
│  ├── 历史最佳轨迹追踪 (应对非单调收敛)                  │
│  └── 详细日志 (方便调试)                               │
└──────────────────────────────────────────────────────┘
```

**至此，核心算法已全部完成！** 接下来的章节将展示如何将这个求解器应用于自动驾驶场景。
