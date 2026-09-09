# Chapter 2: 有限时域 LQR

> **对应代码**：Phase 2  
> **目标**：理解 Bellman 递推和 Riccati 方程，实现从"给定控制"到"求解最优控制"的跨越

---

## 2.1 从"评估"到"优化"

在 Chapter 1 中，我们只能做"评估"——给定控制序列，计算轨迹和代价。但我们真正想做的是**找到最优的控制序列**。

LQR（Linear Quadratic Regulator，线性二次调节器）是这个问题的**线性特例**，具有解析解。它是理解 iLQR 的基础。

---

## 2.2 LQR 问题定义

### 基础概念

LQR 处理的是**线性动力学 + 二次代价**的特殊情况。本项目实现的是**参考跟踪（reference-tracking）**形式的 LQR：

$$
\min_{u_0, \ldots, u_{N-1}} \quad \frac{1}{2} (x_N - x_{\text{ref}})^T Q_f (x_N - x_{\text{ref}}) + \frac{1}{2} \sum_{k=0}^{N-1} \left[ (x_k - x_{\text{ref}})^T Q (x_k - x_{\text{ref}}) + (u_k - u_{\text{ref}})^T R (u_k - u_{\text{ref}}) \right]
$$

$$
\text{s.t.} \quad x_{k+1} = A x_k + B u_k
$$

其中：
- $A \in \mathbb{R}^{n \times n}$ 是状态转移矩阵
- $B \in \mathbb{R}^{n \times m}$ 是控制输入矩阵
- $Q \succeq 0$ 是状态代价矩阵（半正定）
- $R \succ 0$ 是控制代价矩阵（正定）
- $Q_f \succeq 0$ 是终端代价矩阵
- $x_{\text{ref}}$ 是状态参考点（目标状态），$u_{\text{ref}}$ 是控制参考点

> **参考跟踪 vs 调节器形式**：当 $x_{\text{ref}} = 0, u_{\text{ref}} = 0$ 时退化为标准调节器形式。后续推导中，Riccati 递推（求增益 $K_k$）与参考点无关；参考点只影响控制律的应用：$u_k = u_{\text{ref}} + K_k (x_k - x_{\text{ref}})$。

对应代码中的问题定义（`include/lqr/finite_horizon_lqr_solver.hpp`）：

```cpp
struct FiniteHorizonLQRProblem {
  Matrix A;                    // 状态转移矩阵 (n×n)
  Matrix B;                    // 控制输入矩阵 (n×m)
  Matrix Q;                    // 状态代价矩阵 (n×n)
  Matrix R;                    // 控制代价矩阵 (m×m)
  Matrix Qf;                   // 终端代价矩阵 (n×n)
  Vector state_reference;      // 状态参考点
  Vector control_reference;    // 控制参考点
  int horizon = 0;             // 时域长度 N
};
```

---

## 2.3 动态规划与 Bellman 方程

### 基础概念：动态规划

动态规划的核心思想是：**最优策略的子策略也是最优的**（最优性原理）。

如果我们定义**值函数（Value Function）**为从第 $k$ 步开始到终点的最小代价：

$$
V_k(x_k) = \min_{u_k, \ldots, u_{N-1}} \left[ \frac{1}{2} x_N^T Q_f x_N + \sum_{i=k}^{N-1} \frac{1}{2}(x_i^T Q x_i + u_i^T R u_i) \right]
$$

那么 Bellman 方程告诉我们：

$$
V_k(x_k) = \min_{u_k} \left[ \frac{1}{2}(x_k^T Q x_k + u_k^T R u_k) + V_{k+1}(A x_k + B u_k) \right]
$$

含义：**第 $k$ 步的最优代价 = 当前代价 + 从下一状态出发的最优代价**。

### 递推方向

```
终端:                  V_N(x_N) = ½ x_N^T Q_f x_N    ← 已知！

递推:  V_{N-1} ← V_N    V_{N-2} ← V_{N-1}    ...    V_0 ← V_1

方向:  ←────────────────────────────────────────────── 从后往前
```

这种从后往前的递推就是 **backward pass（反向传播）**。

---

## 2.4 Riccati 方程推导

### 假设值函数是二次形式

对于 LQR 问题，值函数恰好是状态的二次函数：

$$
V_k(x_k) = \frac{1}{2} x_k^T P_k x_k
$$

其中 $P_k$ 是 Riccati 矩阵（对称正定），终端条件为 $P_N = Q_f$。

### 定义 Action-Value 函数

将 $V_k$ 中被最小化的部分称为 **Action-Value 函数**（Q 函数）：

$$
\mathcal{Q}(x_k, u_k) = \frac{1}{2} x_k^T Q x_k + \frac{1}{2} u_k^T R u_k + \frac{1}{2}(A x_k + B u_k)^T P_{k+1} (A x_k + B u_k)
$$

### 求最优控制

对 $u_k$ 求导并令其为零：

$$
\frac{\partial \mathcal{Q}}{\partial u_k} = R u_k + B^T P_{k+1}(A x_k + B u_k) = 0
$$

解得最优控制：

$$
u_k^* = -(R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A \, x_k = K_k \, x_k
$$

其中 $K_k$ 是**时变反馈增益矩阵**。

### Riccati 递推

将 $u_k^*$ 代回值函数，得到 $P_k$ 的递推公式：

$$
P_k = Q + A^T P_{k+1}(A + B K_k)
$$

完整的 Riccati 方程展开形式：

$$
P_k = Q + A^T P_{k+1} A - A^T P_{k+1} B (R + B^T P_{k+1} B)^{-1} B^T P_{k+1} A
$$

---

## 2.5 算法流程

```
┌──────────────────────────────────────────────────────┐
│              有限时域 LQR 求解流程                      │
│                                                      │
│  Step 1: 初始化                                       │
│    P_N = Q_f                                         │
│                                                      │
│  Step 2: Backward Pass (从 k=N-1 到 k=0)             │
│    ┌────────────────────────────────────────────┐    │
│    │  S = R + Bᵀ P_{k+1} B                     │    │
│    │  K_k = -S⁻¹ Bᵀ P_{k+1} A                  │    │
│    │  P_k = Q + Aᵀ P_{k+1} (A + B K_k)         │    │
│    └────────────────────────────────────────────┘    │
│                                                      │
│  Step 3: Forward Simulate (从 k=0 到 k=N-1)          │
│    ┌────────────────────────────────────────────┐    │
│    │  u_k = u_ref + K_k (x_k - x_ref)           │    │
│    │  x_{k+1} = A x_k + B u_k                   │    │
│    └────────────────────────────────────────────┘    │
│                                                      │
│  输出: 最优轨迹 {x₀,...,x_N} 和增益 {K₀,...,K_{N-1}} │
└──────────────────────────────────────────────────────┘
```

---

## 2.6 代码实现

### Backward Pass：Riccati 递推

```cpp
// src/lqr/finite_horizon_lqr_solver.cpp
void FiniteHorizonLQRSolver::Solve() {
  // Step 1: 终端条件
  riccati_matrices_.back() = problem_.Qf;    // P_N = Q_f

  // Step 2: 从 k=N-1 递推到 k=0
  for (int k = problem_.horizon - 1; k >= 0; --k) {
    const Matrix& P_next = riccati_matrices_[k + 1];          // P_{k+1}

    // S = R + Bᵀ P_{k+1} B
    const Matrix S = problem_.R + problem_.B.transpose() * P_next * problem_.B;

    // K_k = -S⁻¹ Bᵀ P_{k+1} A
    const Matrix rhs = problem_.B.transpose() * P_next * problem_.A;
    feedback_gains_[k] = -S.ldlt().solve(rhs);                // 用 LDLT 分解求解

    // P_k = Q + Aᵀ P_{k+1} (A + B K_k)
    riccati_matrices_[k] =
        problem_.Q + problem_.A.transpose() * P_next * (problem_.A + problem_.B * feedback_gains_[k]);
  }
  is_solved_ = true;
}
```

### Forward Simulate：闭环仿真

```cpp
Trajectory FiniteHorizonLQRSolver::Simulate(const Vector& initial_state) const {
  Trajectory trajectory(problem_.StateDim(), problem_.ControlDim(), problem_.horizon);
  trajectory.State(0) = initial_state;

  for (int k = 0; k < problem_.horizon; ++k) {
    // 反馈控制律: u_k = u_ref + K_k (x_k - x_ref)
    trajectory.Control(k) = Control(trajectory.State(k), k);

    // 线性动力学: x_{k+1} = A x_k + B u_k
    trajectory.State(k + 1) = problem_.A * trajectory.State(k)
                             + problem_.B * trajectory.Control(k);
  }
  return trajectory;
}

Vector FiniteHorizonLQRSolver::Control(const Vector& state, int k) const {
  const Vector state_error = state - problem_.state_reference;
  return problem_.control_reference + feedback_gains_[k] * state_error;
}
```

---

## 2.7 LQR 与 iLQR 的关系

LQR 是 iLQR 的"内核"。两者的关键区别：

| 特性 | LQR | iLQR |
|------|-----|------|
| 动力学 | 线性 $x_{k+1} = Ax + Bu$ | 非线性 $x_{k+1} = f(x,u)$ |
| 代价 | 二次 | 任意（用二次近似） |
| 求解方式 | 一次 backward + forward | **迭代**：多次 backward + forward |
| Jacobian | $A, B$ 已知 | 需要在每次迭代时重新计算 |

iLQR 的核心思想：在当前轨迹处将非线性问题**局部线性化/二次化**，得到一个 LQR 子问题，求解后更新轨迹，如此反复迭代。

```
  iLQR = 迭代地做以下事情:
  ┌──────────────────────────────────────────┐
  │ 1. 在当前轨迹处线性化动力学 → A_k, B_k    │
  │ 2. 在当前轨迹处二次化代价 → Q, R 等       │
  │ 3. 用 LQR 思路求解反馈增益 K_k, d_k       │  ← 这一步和 LQR 几乎一样！
  │ 4. 用增益更新轨迹                         │
  │ 5. 若未收敛，回到 1                       │
  └──────────────────────────────────────────┘
```

---

## 2.8 本章小结

```
┌──────────────────────────────────────────────────────┐
│                 本章知识结构                           │
│                                                      │
│  问题: 线性动力学 + 二次代价                           │
│     │                                                │
│     ▼                                                │
│  值函数假设: V_k(x) = ½ xᵀ P_k x  (二次形式)         │
│     │                                                │
│     ▼                                                │
│  Bellman 方程 → Riccati 递推                          │
│     │           P_N = Q_f                            │
│     │           P_k = Q + Aᵀ P_{k+1}(A + BK_k)      │
│     │                                                │
│     ▼                                                │
│  最优反馈: u_k* = K_k x_k                            │
│     │      K_k = -(R + Bᵀ P_{k+1} B)⁻¹ Bᵀ P_{k+1} A│
│     │                                                │
│     ▼                                                │
│  Backward Pass (求 K_k) → Forward Simulate (求 x_k)  │
└──────────────────────────────────────────────────────┘
```

**核心收获**：
- Backward pass + Forward simulate 的"双向"结构，是后续 iLQR 和 AL-iLQR 的核心骨架
- LQR 反馈增益 $K_k$ 的求解方式（Riccati 递推），将在 iLQR 中以几乎相同的形式出现
