# 附录：论文步骤与代码实现对照

> **目标**：按论文中的算法步骤，逐项对照本项目代码实现，并明确标注哪些部分与论文一致、哪些是工程化改写、哪些尚未实现。  
> **参考论文**：`paper/AL_iLQR_Tutorial.pdf`、`paper/ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf`

---

## A.1 阅读说明

这份附录不再按源码目录或开发阶段组织，而是直接按照论文中的算法步骤展开。每一步都包含四部分：

- **论文步骤**：论文中的核心数学形式或算法动作
- **代码对应**：本项目中负责这一动作的类、函数和文件
- **对齐情况**：
  - `已对齐`：实现思路与论文主线一致
  - `工程化改写`：目标一致，但实现方式与论文推导不完全相同
  - `未实现`：论文中明确出现，但当前仓库没有实现
  - `项目扩展`：代码确实存在，但属于本项目应用层设计，不是论文主线步骤
- **说明**：帮助读者理解差异来源及影响

当前仓库实现的是 **AL-iLQR 主线**，而不是完整 ALTRO。也就是说：

- 已实现：离散最优控制问题建模、LQR、iLQR、增广拉格朗日、AL-iLQR 外层循环、自动驾驶 demo 场景
- 未实现：ALTRO 中的 square-root backward pass、infeasible-state initialization、projection polishing、minimum-time 扩展等

---

## A.2 论文主线总览

两篇论文的公共主线可以概括为：

$$
\text{Constrained OCP}
\rightarrow
\text{Augmented Lagrangian subproblem}
\rightarrow
\text{iLQR inner solve}
\rightarrow
\text{dual / penalty update}
\rightarrow
\text{repeat}
$$

其中离散时间约束最优控制问题写成：

$$
\min_{x_{0:N},\,u_{0:N-1}}
\ell_N(x_N) + \sum_{k=0}^{N-1}\ell_k(x_k,u_k,\Delta t)
$$

$$
\text{s.t.}\quad
x_{k+1} = f(x_k,u_k,\Delta t),\quad
g_k(x_k,u_k)\le 0,\quad
h_k(x_k,u_k)=0
$$

本项目正是围绕这条主线实现的，只是实现细节更偏“教学版 + 工程简化版”。

---

## A.3 Step 1：建立离散最优控制问题

### 论文步骤

两篇论文都从离散时间 OCP 出发：

$$
x_{k+1} = f(x_k, u_k, \Delta t)
$$

$$
J = \ell_N(x_N) + \sum_{k=0}^{N-1}\ell_k(x_k,u_k,\Delta t)
$$

并在其上叠加阶段约束与终端约束。

### 代码对应

- `include/problems/optimal_control_problem.hpp`
- `src/problems/optimal_control_problem.cpp`
- `include/problems/constrained_optimal_control_problem.hpp`
- `src/problems/constrained_optimal_control_problem.cpp`

核心对象关系：

- `OptimalControlProblem`：无约束 OCP
- `ConstrainedOptimalControlProblem`：在 `OptimalControlProblem` 基础上附加约束

### 对齐情况

`已对齐`

### 说明

本项目的抽象与论文问题定义是一致的：

- 动力学单独封装为 `DynamicsModel`
- 代价单独封装为 `CostFunction`
- 约束单独封装为 `ConstraintFunction`
- 约束问题由 `ConstrainedOptimalControlProblem` 统一持有

但当前项目默认把时间步长 `\Delta t` 当作固定参数，不像 ALTRO 论文那样还讨论了把 `\Delta t` 作为优化变量的扩展。

---

## A.4 Step 2：前向 rollout，隐式满足动力学约束

### 论文步骤

AL-iLQR / iLQR 采用 indirect shooting 思路：状态不是自由优化变量，而是由初始状态和控制序列前向仿真得到。

$$
x_{k+1} = f(x_k, u_k, \Delta t),\quad k=0,\dots,N-1
$$

### 代码对应

- `OptimalControlProblem::Rollout(...)`
- `src/problems/optimal_control_problem.cpp`

```cpp
trajectory.State(0) = initial_state;
for (int k = 0; k < horizon_; ++k) {
  trajectory.Control(k) = control_sequence[k];
  trajectory.State(k + 1) =
      dynamics_->NextState(trajectory.State(k), trajectory.Control(k), dt_);
}
```

### 对齐情况

`已对齐`

### 说明

这一步和论文完全一致：系统轨迹通过动力学显式前向传播得到，天然满足动力学约束。

---

## A.5 Step 3：LQR 背景与有限时域 Riccati 递推

### 论文步骤

论文先用有限时域 LQR 作为 iLQR / DDP 的背景。标准离散 LQR 形式为：

$$
\min_{u_0,\dots,u_{N-1}}
\frac{1}{2}x_N^T Q_f x_N
+
\frac{1}{2}\sum_{k=0}^{N-1}\left(x_k^T Q x_k + u_k^T R u_k\right)
$$

$$
\text{s.t.}\quad x_{k+1}=Ax_k+Bu_k
$$

对应 Riccati 递推：

$$
K_k = -(R + B^T P_{k+1}B)^{-1}B^T P_{k+1}A
$$

$$
P_k = Q + A^T P_{k+1}(A + BK_k)
$$

### 代码对应

- `include/lqr/finite_horizon_lqr_solver.hpp`
- `src/lqr/finite_horizon_lqr_solver.cpp`

### 对齐情况

`已对齐`

### 说明

`FiniteHorizonLQRSolver` 的 backward pass 就是在做上述 Riccati 递推。  
和论文相比，代码还支持参考点跟踪：

$$
u_k = u_{\text{ref}} + K_k(x_k - x_{\text{ref}})
$$

这属于论文主线的直接工程化扩展，不改变核心结构。

---

## A.6 Step 4：在当前轨迹附近做局部展开

### 论文步骤

iLQR / AL-iLQR 的关键是对当前 nominal trajectory 做局部近似。对动力学做一阶线性化：

$$
\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k
$$

其中

$$
A_k = \frac{\partial f}{\partial x}\bigg|_{x_k,u_k},
\qquad
B_k = \frac{\partial f}{\partial u}\bigg|_{x_k,u_k}
$$

对代价做二阶展开，得到：

$$
\ell_x,\ \ell_u,\ \ell_{xx},\ \ell_{ux},\ \ell_{uu}
$$

### 代码对应

- `ILQRSolver::DynamicsJacobianState(...)`
- `ILQRSolver::DynamicsJacobianControl(...)`
- `ILQRSolver::StageCostExpansion(...)`
- `ILQRSolver::TerminalCostExpansion(...)`
- `ILQRSolver::UpdateExpansions(...)`
- `src/ilqr/ilqr_solver.cpp`

### 对齐情况

`工程化改写`

### 说明

思路与论文一致，但实现方式不同：

1. 论文写的是解析形式

$$
A_k = \frac{\partial f}{\partial x},\quad
B_k = \frac{\partial f}{\partial u}
$$

2. 本项目使用中心差分数值近似：

$$
\frac{\partial f}{\partial x_i}
\approx
\frac{f(x+\varepsilon e_i,u)-f(x-\varepsilon e_i,u)}{2\varepsilon}
$$

以及类似的代价梯度 / Hessian 中心差分。

因此，当前实现更准确地说是：

- **数值差分版 iLQR**
- 不是解析导数版
- 也不是 full DDP

---

## A.7 Step 5：构造 action-value 二次模型

### 论文步骤

在 backward pass 中，论文定义 action-value 展开：

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

在 AL-iLQR 里，论文还会把约束线性化项并入这些表达式。

### 代码对应

- `ILQRSolver::BackwardPass(...)`
- `src/ilqr/ilqr_solver.cpp`

```cpp
const Vector Qx = expansion.lx + A.transpose() * Vx;
const Vector Qu = expansion.lu + B.transpose() * Vx;
const Matrix Qxx = expansion.lxx + A.transpose() * Vxx * A;
const Matrix Qux = expansion.lux + B.transpose() * Vxx * A;
const Matrix Quu = expansion.luu + B.transpose() * Vxx * B;
```

### 对齐情况

`已对齐`

### 说明

这部分和 tutorial / ALTRO 背景章节中的 AL-iLQR 公式是一致的。

需要注意的是，当前项目里 `expansion.lx`、`expansion.lu`、`expansion.lxx` 等量并不区分“原始代价”和“AL 项贡献”，因为它们都是通过对当前 stage cost 整体做数值差分得到的。

---

## A.8 Step 6：求解反馈项与前馈项

### 论文步骤

论文中的控制修正写为：

$$
\delta u_k^\ast = K_k \delta x_k + d_k
$$

其中

$$
K_k = -(Q_{uu} + \rho I)^{-1}Q_{ux}
$$

$$
d_k = -(Q_{uu} + \rho I)^{-1}Q_u
$$

### 代码对应

- `ILQRSolver::BackwardPass(...)`
- `src/ilqr/ilqr_solver.cpp`

```cpp
const Matrix Quu_reg =
    Quu + regularization * Matrix::Identity(problem_.ControlDim(), problem_.ControlDim());

Eigen::LLT<Matrix> llt(Quu_reg);
feedback_gains_[k] = -llt.solve(Qux);
feedforward_terms_[k] = -llt.solve(Qu);
```

### 对齐情况

`已对齐`

### 说明

这部分和论文公式基本一一对应。当前代码使用 `LLT` 做正定矩阵求解，这相当于标准实现中的 Cholesky 路线。

---

## A.9 Step 7：更新值函数与期望下降量

### 论文步骤

论文给出的值函数递推为：

$$
P_k = Q_{xx} + K_k^T Q_{uu} K_k + K_k^T Q_{ux} + Q_{xu}K_k
$$

$$
p_k = Q_x + K_k^T Q_{uu} d_k + K_k^T Q_u + Q_{xu} d_k
$$

并累计期望下降量：

$$
\Delta V_k = d_k^T Q_u + \frac{1}{2} d_k^T Q_{uu} d_k
$$

### 代码对应

- `ILQRSolver::BackwardPass(...)`
- `src/ilqr/ilqr_solver.cpp`

### 对齐情况

`已对齐`

### 说明

代码中：

- `Vx` 对应论文里的 $p_k$
- `Vxx` 对应论文里的 $P_k$
- `expected_linear` 和 `expected_quadratic` 对应期望下降量中的一阶、二阶部分

当前实现还额外做了对称化：

$$
V_{xx} \leftarrow \frac{1}{2}(V_{xx}+V_{xx}^T)
$$

这是常见工程化处理，用于抑制数值差分误差。

---

## A.10 Step 8：forward pass 与 line search

### 论文步骤

论文中的 forward pass 使用闭环策略更新控制：

$$
\bar{u}_k = u_k + K_k(\bar{x}_k - x_k) + \alpha d_k
$$

$$
\bar{x}_{k+1} = f(\bar{x}_k,\bar{u}_k)
$$

并计算实际下降量和期望下降量之比：

$$
z =
\frac{J(X,U)-J(\bar{X},\bar{U})}{-\Delta V(\alpha)}
$$

其中

$$
\Delta V(\alpha)
=
\alpha \Delta V_1 + \alpha^2 \Delta V_2
$$

### 代码对应

- `ILQRSolver::ForwardPass(...)`
- `src/ilqr/ilqr_solver.cpp`

### 对齐情况

`已对齐`

### 说明

代码里直接实现了 backtracking line search：

- 初始 `\alpha = 1`
- 若不接受则按固定因子衰减
- 只有在 `candidate_cost < current_cost` 且 ratio 落在接受区间时才接受

这与 tutorial 中的标准 iLQR 线搜索结构一致。

---

## A.11 Step 9：正则化与数值稳定性

### 论文步骤

论文中在求解控制修正时加入正则化：

$$
Q_{uu} + \rho I
$$

并根据 backward pass / forward pass 是否成功自适应调整 `\rho`。

### 代码对应

- `include/ilqr/ilqr_solver.hpp`
- `ILQRSolver::Solve(...)`
- `src/ilqr/ilqr_solver.cpp`

### 对齐情况

`已对齐`

### 说明

当前代码采用的策略是：

- backward pass 失败：增大 `\rho`
- forward pass 失败：增大 `\rho`
- 接受新轨迹：减小 `\rho`
- 超过上限：提前返回当前轨迹

这和论文描述的 regularization 主线一致，只是具体系数由项目配置决定。

---

## A.12 Step 10：统一约束建模

### 论文步骤

论文把约束统一写成：

$$
g_k(x_k,u_k)\le 0,\qquad h_k(x_k,u_k)=0
$$

并在 AL 框架中统一处理 inequality / equality。

### 代码对应

- `include/constraints/constraint_function.hpp`
- `src/constraints/constraint_function.cpp`
- `include/problems/constrained_optimal_control_problem.hpp`
- `src/problems/constrained_optimal_control_problem.cpp`

### 对齐情况

`已对齐`

### 说明

本项目的 `ConstraintFunction` 正是对论文中一般约束函数的直接工程抽象：

- `Type()` 区分等式 / 不等式
- `Evaluate(x,u)` 返回向量约束值
- `OutputDim()` 允许一个约束函数输出多维约束

---

## A.13 Step 11：构造增广拉格朗日

### 论文步骤

对于约束问题：

$$
\min f(x)\quad \text{s.t.}\quad c(x)\{\le,=\}0
$$

tutorial 给出的增广拉格朗日形式是：

$$
\mathcal{L}_A
=
f(x) + \lambda^T c(x) + \frac{1}{2} c(x)^T I_\mu c(x)
$$

对于等式约束，单项形式可以写成：

$$
\lambda_i c_i + \frac{1}{2}\mu_i c_i^2
$$

对于不等式约束，项目采用的写法与 `AL_iLQR_Tutorial.pdf` 一致（注意 ALTRO 论文主体使用的是 Conic 形式，两者数学等价但符号不同，详见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)）：

$$
\frac{1}{2\mu_i}
\left(
\max(0,\lambda_i+\mu_i c_i)^2 - \lambda_i^2
\right)
$$

### 代码对应

- `AugmentedLagrangianConstraintData`
- `AugmentedLagrangianKnotCost::ConstraintAugmentedTerm(...)`
- `src/al/augmented_lagrangian_cost.cpp`

### 对齐情况

`已对齐`

### 说明

项目里把每个约束的 AL 状态单独保存为：

- `lambda`
- `mu`
- `penalty_scaling`

然后逐个约束求和，叠加到原始 stage cost / terminal cost 上。

---

## A.14 Step 12：把带约束问题转成无约束子问题

### 论文步骤

论文中的 AL-iLQR 外层思想是：固定 `\lambda,\mu` 后，求解当前的无约束子问题

$$
\min_{X,U}\ \mathcal{L}_A(X,U,\lambda,\mu)
$$

### 代码对应

- `AugmentedLagrangianProblem`
- `src/al/augmented_lagrangian_cost.cpp`

其具体做法是：

1. 为每个阶段创建 `AugmentedLagrangianKnotCost`
2. 为终端创建 `AugmentedLagrangianKnotCost`
3. 用这些包装后的 cost 重新构造一个 `OptimalControlProblem`

### 对齐情况

`已对齐`

### 说明

这一步是本项目实现中最关键的“桥”：

$$
\text{Constrained OCP}
\rightarrow
\text{AL-wrapped unconstrained OCP}
\rightarrow
\text{iLQR solve}
$$

这和论文的“固定 dual / penalty 后求解无约束子问题”是完全一致的。

---

## A.15 Step 13：更新对偶变量

### 论文步骤

标准 AL 更新为：

等式约束：

$$
\lambda_i^+ = \lambda_i + \mu_i c_i(x^\ast,u^\ast)
$$

不等式约束：

$$
\lambda_i^+ = \max\left(0,\lambda_i + \mu_i c_i(x^\ast,u^\ast)\right)
$$

### 代码对应

- `AugmentedLagrangianKnotCost::UpdateDuals(...)`
- `AugmentedLagrangianProblem::UpdateDuals(...)`
- `src/al/augmented_lagrangian_cost.cpp`

### 对齐情况

`已对齐`

### 说明

当前实现对 stage constraints 和 terminal constraints 都做了相同逻辑，只是 terminal 使用空控制向量。

---

## A.16 Step 14：更新惩罚参数

### 论文步骤

标准 AL / AL-iLQR 写法通常是单调增大惩罚参数：

$$
\mu_i^+ = \phi \mu_i,\qquad \phi > 1
$$

### 代码对应

- `AugmentedLagrangianKnotCost::UpdatePenalties()`
- `AugmentedLagrangianProblem::UpdatePenalties()`
- `src/al/augmented_lagrangian_cost.cpp`

### 对齐情况

`已对齐`

### 说明

如果只看 `UpdatePenalties()` 本身，项目与论文是一致的，都是逐元素：

$$
\mu \leftarrow \phi \mu
$$

真正的差异不在“怎么更新”，而在“什么时候触发更新”，见下一步。

---

## A.17 Step 15：AL-iLQR 外层循环

### 论文步骤

标准 AL-iLQR 外层循环可以写成：

1. 固定 `\lambda,\mu`
2. 最小化当前增广拉格朗日子问题
3. 更新 `\lambda`
4. 更新 `\mu`
5. 检查约束收敛
6. 若未收敛则继续

### 代码对应

- `ALILQRSolver::Solve(...)`
- `src/al/al_ilqr_solver.cpp`

### 对齐情况

`工程化改写`

### 说明

主线和论文一致，但当前项目做了两个工程化修改：

1. **历史最佳轨迹回退**

代码会持续记录：

$$
\text{best\_violation} = \min_t \text{violation}^{(t)}
$$

如果最后一次轨迹比历史最佳更差，**且 `options_.return_best_trajectory` 为 `true`（默认值）**，则返回历史最佳轨迹而不是最后一次轨迹。这不是论文主线中的必要部分，但对工程稳定性有帮助。

2. **条件性惩罚更新 heuristic**

论文标准讲法通常是“每轮外层后更新惩罚”或“按统一 schedule 增大 `\mu`”。  
本项目不是这样，而是只有在“进展不足”时才更新惩罚。

代码判据为：

$$
\text{insufficient\_progress\_since\_penalty}
\iff
\text{violation} >
\text{penalty\_reference\_violation}\cdot r
$$

以及

$$
\text{insufficient\_progress\_since\_last\_outer}
\iff
\text{outer\_iter} > 0 \;\wedge\;
\text{violation} >
\text{previous\_outer\_violation}\cdot s
$$

其中代码里

$$
s = \max(r, 0.95)
$$

注意第二个判据仅在 `outer_iter > 0` 时才生效（第一次外层迭代没有 `previous_outer_violation` 可比较）。见 `src/al/al_ilqr_solver.cpp`。

这个 **条件性惩罚更新** 是本项目自己的 heuristic，不是论文标准 AL 公式。

---

## A.18 Step 16：约束收敛判据

### 论文步骤

论文中最终会检查约束是否达到目标容差，典型形式是：

$$
\max_i \text{violation}_i \le \varepsilon
$$

### 代码对应

- `ConstrainedOptimalControlProblem::MaxViolation(...)`
- `AugmentedLagrangianProblem::MaxViolation(...)`
- `ALILQRSolver::Solve(...)`

### 对齐情况

`已对齐`

### 说明

项目中统一使用 max violation 作为收敛指标，这与论文主线一致。

---

## A.19 Step 17：自动驾驶场景建模

### 论文步骤

两篇论文的核心重点是求解器，不是本项目中的自动驾驶 demo 场景。也就是说：

- 参考线跟踪
- 道路边界约束
- 速度限制约束
- 多圆碰撞约束
- 静态 / 动态障碍物场景工厂

这些都属于**项目应用层扩展**，不是论文主线的一部分。

### 代码对应

- `src/autodrive/straight_reference_line.cpp`
- `src/autodrive/lane_tracking_cost.cpp`
- `src/autodrive/road_boundary_constraint.cpp`
- `src/autodrive/speed_limit_constraint.cpp`
- `src/autodrive/multi_circle_vehicle_obstacle_constraint.cpp`
- `src/autodrive/demo_scenario.cpp`

### 对齐情况

`项目扩展`

### 说明

这部分不是“和论文不一致”，而是“论文没有限定具体应用，本项目选了一个简化自动驾驶场景来承载 AL-iLQR 求解器”。

---

## A.20 ALTRO 中额外提出、但本项目未实现的步骤

这一节专门列出 `paper/ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf` 中出现，但当前仓库没有实现的部分。

### A.20.1 Square-root backward pass

#### 论文步骤

ALTRO 为了改善大 penalty 下的数值病态，引入了 square-root backward pass，用 Cholesky / QR 因子递推，而不是直接递推 `P_k`。

#### 代码对应

无

#### 对齐情况

`未实现`

#### 说明

当前项目使用的是标准矩阵形式：

$$
Q_{uu} + \rho I
$$

然后通过 `LLT` 直接求解，不包含 square-root Riccati 递推。

---

### A.20.2 Infeasible-state initialization

#### 论文步骤

ALTRO 讨论了如何用 infeasible state trajectory 来初始化 DDP / iLQR 变体。

#### 代码对应

无

#### 对齐情况

`未实现`

#### 说明

当前项目始终从：

- 初始状态 `x_0`
- 初始控制序列 `U_0`

出发，然后通过 rollout 得到动态可行的 nominal trajectory。

---

### A.20.3 Projection polishing / solution polishing

#### 论文步骤

ALTRO 的第二阶段不是继续粗略 AL 收敛，而是使用 active-set projection / Newton polishing 进一步提高约束满足精度。

#### 代码对应

无

#### 对齐情况

`未实现`

#### 说明

当前仓库在 AL-iLQR 外层收敛后直接返回结果，没有第二阶段 polishing。

---

### A.20.4 Minimum-time 扩展

#### 论文步骤

ALTRO 讨论了最小时间问题，即把 `\Delta t` 或总时间也纳入优化。

#### 代码对应

无

#### 对齐情况

`未实现`

#### 说明

当前项目里 `dt` 固定，`horizon` 固定，不做 minimum-time 优化。

---

## A.21 一页总结

### 已与论文主线对齐的部分

- 离散 OCP 建模
- indirect rollout
- 有限时域 LQR
- iLQR 局部线性化 / 二次化主线
- backward pass
- forward pass + line search
- regularization
- 统一约束建模
- 增广拉格朗日代价构造
- dual update
- max violation 收敛判据

### 与论文目标一致、但实现方式工程化改写的部分

- 导数计算：解析导数改为数值差分
- AL 项进入 iLQR 的方式：显式约束导数展开改为“包装代价后整体差分”
- AL 外层循环：加入 best-trajectory 返回策略（受 `return_best_trajectory` 选项控制）
- 惩罚更新触发时机：采用项目自定义的条件性更新 heuristic
- 内层求解器生命周期：每次外层迭代重建 `ILQRSolver`（论文隐含复用）
- AL 约束公式形式：使用标准形式而非 ALTRO 的 Conic 形式（数学等价）

### ALTRO 论文中有、当前仓库未实现的部分

- square-root backward pass
- infeasible-state initialization
- projection polishing
- minimum-time 扩展

---

## A.22 建议读法

如果你想按“论文 → 代码”顺序理解本项目，推荐按下面顺序阅读：

1. 先看本附录的 `A.3` 到 `A.17`，建立论文步骤和代码对象的一一映射
2. 再回到 [chapter3_ilqr.md](chapter3_ilqr.md)、[chapter5_augmented_lagrangian.md](chapter5_augmented_lagrangian.md)、[chapter6_al_ilqr.md](chapter6_al_ilqr.md) 看教学化解释
3. 最后读 [chapter7_autodrive.md](chapter7_autodrive.md)，理解求解器如何落到自动驾驶 demo 场景

这样阅读最不容易把“论文标准算法”和“项目工程化实现细节”混在一起。
