# Chapter 3: 无约束 iLQR 求解器

> **对应代码**：Phase 3-4  
> **目标**：将 LQR 推广到非线性系统，实现带正则化和线搜索的稳定 iLQR

---

## 3.1 从 LQR 到 iLQR

### 基础概念

LQR 只能处理线性动力学。但现实中的动力学（如自行车模型）都是非线性的：

$$
x_{k+1} = f(x_k, u_k, \Delta t) \quad \text{（非线性）}
$$

iLQR（iterative Linear Quadratic Regulator）的核心思想是：**在当前轨迹的每个 knot point 处，对非线性问题做局部线性/二次近似，转化为 LQR 子问题，然后迭代求解**。

> **实现说明**：本项目实现的是 **iLQR**（只用一阶动力学线性化 $A_k, B_k$），不是 full DDP（Differential Dynamic Programming，需要动力学二阶展开 $f_{xx}, f_{xu}, f_{uu}$）。所有 Jacobian 和 Hessian 均通过**数值中心差分**计算，不是解析导数。这简化了实现，代价是计算量较大且精度受差分步长 $\epsilon$ 影响。关于与论文的完整差异清单，参见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)。

```
   iLQR 迭代过程:

   初始轨迹 (可能很差)
       │
       ▼
   ┌─────────────────────────────────────┐
   │ ① 展开: 在当前轨迹处计算 A,B 和代价导数 │ ← 把非线性问题局部线性化
   │ ② Backward Pass: 求反馈增益 K_k, d_k  │ ← 类似 LQR 的 Riccati 递推
   │ ③ Forward Pass: 用增益仿真新轨迹       │ ← 带线搜索的闭环 rollout
   │ ④ 收敛? → 否 → 回到 ①                │
   └─────────────────────────────────────┘
       │ 是
       ▼
   最优轨迹
```

---

## 3.2 局部展开（Expansion）

### 3.2.1 动力学线性化

在 knot point $k$ 处，将非线性动力学做一阶 Taylor 展开：

$$
f(x_k + \delta x, u_k + \delta u) \approx f(x_k, u_k) + A_k \delta x + B_k \delta u
$$

其中 Jacobian 矩阵：

$$
A_k = \frac{\partial f}{\partial x}\bigg|_{x_k, u_k} \in \mathbb{R}^{n \times n}, \quad B_k = \frac{\partial f}{\partial u}\bigg|_{x_k, u_k} \in \mathbb{R}^{n \times m}
$$

本项目使用**中心差分**数值计算 Jacobian：

$$
\frac{\partial f}{\partial x_i} \approx \frac{f(x + \epsilon e_i, u) - f(x - \epsilon e_i, u)}{2\epsilon}
$$

对应代码（`src/ilqr/ilqr_solver.cpp`）：

```cpp
Matrix ILQRSolver::DynamicsJacobianState(const Vector& state, const Vector& control) const {
  const double eps = options_.derivative_epsilon;   // 典型值: 1e-4
  Matrix jac = Matrix::Zero(problem_.StateDim(), problem_.StateDim());
  for (int i = 0; i < problem_.StateDim(); ++i) {
    Vector x_plus = state;
    Vector x_minus = state;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    const Vector f_plus = problem_.Dynamics().NextState(x_plus, control, problem_.TimeStep());
    const Vector f_minus = problem_.Dynamics().NextState(x_minus, control, problem_.TimeStep());
    jac.col(i) = (f_plus - f_minus) / (2.0 * eps);   // 中心差分
  }
  return jac;
}
```

### 3.2.2 代价二次化

同样对代价函数计算梯度和 Hessian：

$$
\ell_x = \frac{\partial \ell}{\partial x}, \quad \ell_u = \frac{\partial \ell}{\partial u}, \quad \ell_{xx} = \frac{\partial^2 \ell}{\partial x^2}, \quad \ell_{ux} = \frac{\partial^2 \ell}{\partial u \partial x}, \quad \ell_{uu} = \frac{\partial^2 \ell}{\partial u^2}
$$

也使用中心差分计算。代码中封装为 `CostExpansion` 结构：

```cpp
struct CostExpansion {
  Vector lx;     // ∂ℓ/∂x   (n×1)
  Vector lu;     // ∂ℓ/∂u   (m×1)
  Matrix lxx;    // ∂²ℓ/∂x² (n×n)
  Matrix lux;    // ∂²ℓ/∂u∂x (m×n)
  Matrix luu;    // ∂²ℓ/∂u² (m×m)
};
```

### 3.2.3 展开的完整流程

```
   对每个 knot point k = 0, 1, ..., N-1:

   ┌─────────────────────────────────────────┐
   │ 动力学 Jacobian:                         │
   │   A_k = ∂f/∂x |_{x_k, u_k}  (n×n)      │
   │   B_k = ∂f/∂u |_{x_k, u_k}  (n×m)      │
   │                                          │
   │ 代价梯度/Hessian:                        │
   │   ℓ_x, ℓ_u, ℓ_{xx}, ℓ_{ux}, ℓ_{uu}     │
   └─────────────────────────────────────────┘
```

对应代码：

```cpp
void ILQRSolver::UpdateExpansions(const Trajectory& trajectory) {
  for (int k = 0; k < problem_.Horizon(); ++k) {
    A_[k] = DynamicsJacobianState(trajectory.State(k), trajectory.Control(k));
    B_[k] = DynamicsJacobianControl(trajectory.State(k), trajectory.Control(k));
    stage_expansions_[k] = StageCostExpansion(k, trajectory.State(k), trajectory.Control(k));
  }
}
```

---

## 3.3 Backward Pass

### 3.3.1 Action-Value 展开

与 LQR 类似，定义 Q 值展开（对应论文公式 (8)-(12)）：

$$
Q_x = \ell_x + A^T V_x'
$$

$$
Q_u = \ell_u + B^T V_x'
$$

$$
Q_{xx} = \ell_{xx} + A^T V_{xx}' A
$$

$$
Q_{ux} = \ell_{ux} + B^T V_{xx}' A
$$

$$
Q_{uu} = \ell_{uu} + B^T V_{xx}' B
$$

其中 $V_x', V_{xx}'$ 是 $k+1$ 步的值函数梯度和 Hessian。

### 3.3.2 求解增益

最优控制修正量（对应论文公式 (13)）：

$$
\delta u_k^* = K_k \delta x_k + d_k
$$

$$
K_k = -(Q_{uu} + \rho I)^{-1} Q_{ux} \quad \text{（反馈增益）}
$$

$$
d_k = -(Q_{uu} + \rho I)^{-1} Q_u \quad \text{（前馈项）}
$$

> **与 LQR 的区别**：
> - 多了前馈项 $d_k$（因为代价展开有线性项 $Q_u$）
> - 多了正则化 $\rho I$（确保 $Q_{uu}$ 正定，可逆）

### 3.3.3 更新值函数

（对应论文公式 (14)-(16)）：

$$
V_x = Q_x + K^T Q_{uu} d + K^T Q_u + Q_{ux}^T d
$$

$$
V_{xx} = Q_{xx} + K^T Q_{uu} K + K^T Q_{ux} + Q_{ux}^T K
$$

$$
\Delta V_1 = Q_u^T d, \quad \Delta V_2 = \frac{1}{2} d^T Q_{uu} d
$$

### 3.3.4 完整 Backward Pass 代码

```cpp
bool ILQRSolver::BackwardPass(const Trajectory& trajectory,
                              double regularization,
                              double* expected_linear,
                              double* expected_quadratic) {
  // 终端值函数初始化
  const auto [terminal_gradient, terminal_hessian] =
      TerminalCostExpansion(trajectory.State(problem_.Horizon()));
  Vector Vx = terminal_gradient;       // V_x = ∂ℓ_f/∂x
  Matrix Vxx = terminal_hessian;       // V_{xx} = ∂²ℓ_f/∂x²
  *expected_linear = 0.0;
  *expected_quadratic = 0.0;

  // 从 k=N-1 递推到 k=0
  for (int k = problem_.Horizon() - 1; k >= 0; --k) {
    const auto& expansion = stage_expansions_[k];
    const Matrix& A = A_[k];
    const Matrix& B = B_[k];

    // Q 值展开 (公式 (8)-(12))
    const Vector Qx = expansion.lx + A.transpose() * Vx;
    const Vector Qu = expansion.lu + B.transpose() * Vx;
    const Matrix Qxx = expansion.lxx + A.transpose() * Vxx * A;
    const Matrix Qux = expansion.lux + B.transpose() * Vxx * A;
    const Matrix Quu = expansion.luu + B.transpose() * Vxx * B;

    // 正则化 (确保 Quu 正定)
    const Matrix Quu_reg = Quu + regularization * Matrix::Identity(...);

    // Cholesky 分解 (如果失败说明 Quu 不正定)
    Eigen::LLT<Matrix> llt(Quu_reg);
    if (llt.info() != Eigen::Success) {
      return false;   // 需要增大正则化
    }

    // 求增益 (公式 (13))
    feedback_gains_[k] = -llt.solve(Qux);       // K = -Quu⁻¹ Qux
    feedforward_terms_[k] = -llt.solve(Qu);      // d = -Quu⁻¹ Qu

    // 累计期望代价减少量 (公式 (16))
    *expected_linear += Qu.dot(feedforward_terms_[k]);
    *expected_quadratic += 0.5 * feedforward_terms_[k].dot(Quu * feedforward_terms_[k]);

    // 更新值函数 (公式 (14)-(15))
    Vx = Qx + feedback_gains_[k].transpose() * Quu * feedforward_terms_[k]
       + feedback_gains_[k].transpose() * Qu
       + Qux.transpose() * feedforward_terms_[k];
    Vxx = Qxx + feedback_gains_[k].transpose() * Quu * feedback_gains_[k]
        + feedback_gains_[k].transpose() * Qux
        + Qux.transpose() * feedback_gains_[k];
    Vxx = 0.5 * (Vxx + Vxx.transpose());   // 对称化，防止数值误差累积
  }
  return true;
}
```

---

## 3.4 Forward Pass（前向线搜索）

### 3.4.1 闭环 Rollout

用 backward pass 得到的策略 $(K_k, d_k)$ 仿真新轨迹（对应论文公式 (50)-(53)）：

$$
\bar{u}_k = u_k + \alpha \cdot d_k + K_k \cdot (\bar{x}_k - x_k)
$$

$$
\bar{x}_{k+1} = f(\bar{x}_k, \bar{u}_k)
$$

其中 $\alpha \in (0, 1]$ 是线搜索参数，控制步长大小。

```
   Forward Pass 闭环 Rollout:

   x̄₀ = x₀ (固定)
     │
     ├── δx = x̄₀ - x₀ = 0
     ├── ū₀ = u₀ + α·d₀ + K₀·δx
     ├── x̄₁ = f(x̄₀, ū₀)
     │
     ├── δx = x̄₁ - x₁
     ├── ū₁ = u₁ + α·d₁ + K₁·δx
     ├── x̄₂ = f(x̄₁, ū₁)
     │
     └── ...
```

### 3.4.2 线搜索接受条件

计算实际代价减少量与期望减少量的比值（对应论文公式 (54)-(55)）：

$$
z = \frac{J_{\text{old}} - J_{\text{new}}}{-\Delta V(\alpha)}
$$

$$
\Delta V(\alpha) = \alpha \cdot \Delta V_1 + \alpha^2 \cdot \Delta V_2
$$

接受条件：$z \in [\beta_1, \beta_2]$ 且 $J_{\text{new}} < J_{\text{old}}$（典型值 $\beta_1 = 10^{-4}, \beta_2 = 10$）。

如果不接受，缩小步长：$\alpha \leftarrow \gamma \cdot \alpha$（$\gamma = 0.5$）。

### 3.4.3 Forward Pass 代码

```cpp
bool ILQRSolver::ForwardPass(...) {
  double alpha = 1.0;

  for (int iter = 0; iter < options_.line_search_max_iterations; ++iter) {
    // 闭环 rollout
    Trajectory candidate(problem_.StateDim(), problem_.ControlDim(), problem_.Horizon());
    candidate.State(0) = initial_state;

    for (int k = 0; k < problem_.Horizon(); ++k) {
      const Vector dx = candidate.State(k) - nominal_trajectory.State(k);
      candidate.Control(k) = nominal_trajectory.Control(k)
                            + alpha * feedforward_terms_[k]    // α · d_k
                            + feedback_gains_[k] * dx;         // K_k · δx
      candidate.State(k + 1) =
          problem_.Dynamics().NextState(candidate.State(k), candidate.Control(k), ...);
    }

    // 检查接受条件
    const double candidate_cost = EvaluateTrajectoryCost(candidate);
    const double actual_decrease = current_cost - candidate_cost;
    const double expected_decrease = -(alpha * expected_linear + alpha * alpha * expected_quadratic);

    if (expected_decrease > 0.0) {
      const double ratio = actual_decrease / expected_decrease;
      if (candidate_cost < current_cost && ratio >= lower_bound && ratio <= upper_bound) {
        *accepted_trajectory = candidate;    // 接受!
        return true;
      }
    }

    alpha *= options_.line_search_decrease_factor;   // 缩小步长，重试
  }
  return false;   // 所有步长都失败
}
```

---

## 3.5 正则化策略

### 基础概念

正则化是 iLQR 数值稳定性的关键。$\rho$ 越大，行为越像梯度下降（稳定但慢）；$\rho$ 越小，越像 Newton 法（快但可能不稳定）。

```
   ρ 的自适应调整:

   Backward Pass 成功? ──→ 是 ──→ Forward Pass 成功? ──→ 是 ──→ ρ ↓ (减小)
          │                              │
          └── 否 (Quu 不正定)            └── 否 (线搜索失败)
              │                              │
              ▼                              ▼
           ρ ↑ (增大)                     ρ ↑ (增大)
           重试 Backward Pass             重试
```

代码中的自适应策略：

```cpp
// 成功时减小正则化
regularization = std::max(reg_min, regularization / decrease_factor);

// 失败时增大正则化
regularization *= increase_factor;
if (regularization > reg_max) {
  return trajectory;   // 放弃，返回当前最好的结果
}
```

---

## 3.6 iLQR 主循环

```cpp
Trajectory ILQRSolver::Solve(const Vector& initial_state,
                             const std::vector<Vector>& initial_controls) {
  // 初始 rollout
  Trajectory trajectory = problem_.Rollout(initial_state, initial_controls);
  double regularization = options_.regularization_init;

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    bool accepted = false;
    while (!accepted) {
      UpdateExpansions(trajectory);                     // ① 展开

      double expected_linear = 0.0, expected_quadratic = 0.0;
      bool backward_ok = BackwardPass(trajectory,       // ② Backward Pass
                                      regularization,
                                      &expected_linear,
                                      &expected_quadratic);
      if (!backward_ok) {                               // Quu 不正定
        regularization *= increase_factor;
        continue;
      }

      accepted = ForwardPass(initial_state, trajectory, // ③ Forward Pass
                             ...);
      if (accepted) {
        regularization /= decrease_factor;              // 成功: ρ 减小
      } else {
        regularization *= increase_factor;              // 失败: ρ 增大
      }
    }

    if (cost_converged) break;                          // ④ 收敛检查
  }
  return trajectory;
}
```

---

## 3.7 本章小结

```
┌───────────────────────────────────────────────────────────┐
│                  iLQR 完整算法流程                          │
│                                                           │
│  输入: 初始状态 x₀, 初始控制序列 {u₀,...,u_{N-1}}          │
│                                                           │
│  Rollout: x₀ → x₁ → ... → x_N                           │
│                                                           │
│  迭代:                                                    │
│  ┌──────────────────────────────────────────────────┐     │
│  │ ① 展开 (每个 knot point):                        │     │
│  │    ├─ A_k = ∂f/∂x (数值差分)                     │     │
│  │    ├─ B_k = ∂f/∂u (数值差分)                     │     │
│  │    └─ ℓ_x, ℓ_u, ℓ_{xx}, ℓ_{ux}, ℓ_{uu} (数值差分)│     │
│  │                                                  │     │
│  │ ② Backward Pass (k = N-1 到 0):                  │     │
│  │    ├─ Q 值展开: Qx, Qu, Qxx, Qux, Quu           │     │
│  │    ├─ 正则化: Quu + ρI                           │     │
│  │    ├─ Cholesky 分解                              │     │
│  │    ├─ K_k = -Quu⁻¹ Qux, d_k = -Quu⁻¹ Qu        │     │
│  │    └─ 更新 Vx, Vxx                               │     │
│  │                                                  │     │
│  │ ③ Forward Pass:                                  │     │
│  │    ├─ ū_k = u_k + α d_k + K_k δx               │     │
│  │    ├─ x̄_{k+1} = f(x̄_k, ū_k)                    │     │
│  │    └─ 线搜索: α = 1, 0.5, 0.25, ...             │     │
│  │                                                  │     │
│  │ ④ 收敛检查: |J_old - J_new| < tolerance          │     │
│  └──────────────────────────────────────────────────┘     │
│                                                           │
│  输出: 最优轨迹                                            │
└───────────────────────────────────────────────────────────┘
```

**核心收获**：
- iLQR = 迭代地在当前轨迹处做局部 LQR
- 正则化确保数值稳定（$Q_{uu}$ 正定）
- 线搜索确保每步都在下降
- 此时只能处理**无约束**问题；约束处理在 Chapter 4-6 中引入
