# AL-iLQR 求导架构分析与改进方向

## 1. 整体架构

当前 AL-iLQR 的求导采用**统一的中心差分（Central Finite Difference）**策略。所有约束通过增广拉格朗日（AL）罚项融入代价函数后，由 iLQR 对"总代价"统一做数值差分求导。

```
                        ┌─────────────────────────────────┐
                        │     总代价 J(x, u)               │
                        │  = 原始代价 ℓ(x,u)               │
                        │  + Σ AL罚项(λ, μ, c_i(x,u))     │
                        └──────────┬──────────────────────┘
                                   │
                    iLQR 中心差分 (eps = derivative_epsilon)
                                   │
              ┌────────────────────┼────────────────────┐
              ▼                    ▼                    ▼
        动力学 Jacobian       代价梯度              代价 Hessian
        A_k = ∂f/∂x         lx, lu              lxx, luu, lux
        B_k = ∂f/∂u
```

**核心特点**：约束本身只需提供 `Evaluate(x, u) → c(x,u)` 值函数，不需要提供解析 Jacobian。

---

## 2. 需要求导的全部组成部分

### 2.1 动力学模型 — Jacobian A, B

| 导数 | 维度 | 当前方法 | 每步函数调用次数 |
|------|------|----------|-----------------|
| A = ∂f/∂x | 4×4 | 中心差分 | 2n = 8 次 `NextState` |
| B = ∂f/∂u | 4×2 | 中心差分 | 2m = 4 次 `NextState` |

动力学方程（`KinematicBicycleModel`）：

```
x_{k+1} = x_k + dt · v_k · cos(θ_k)
y_{k+1} = y_k + dt · v_k · sin(θ_k)
θ_{k+1} = θ_k + dt · v_k · tan(δ_k) / L
v_{k+1} = v_k + dt · a_k
```

代码位置：`src/ilqr/ilqr_solver.cpp` → `DynamicsJacobianState`, `DynamicsJacobianControl`

### 2.2 原始代价函数 — 梯度与 Hessian

| 代价类型 | 类名 | 形式 |
|----------|------|------|
| 车道跟踪 | `LaneTrackingCost` | 加权二次：横向偏差 + 航向偏差 + 速度偏差 + 控制正则化 |
| 引导跟踪 | `GuidanceTrackingCost` | 加权二次：位置偏差 + 航向偏差 + 速度偏差 |
| 通用二次 | `QuadraticCost` | 标准 (x-x_ref)ᵀQ(x-x_ref) + (u-u_ref)ᵀR(u-u_ref) |

| 导数 | 维度 | 当前方法 | 每步函数调用次数 |
|------|------|----------|-----------------|
| lx (梯度) | 4×1 | 中心差分 | 2n = 8 次 `StageCost` |
| lu (梯度) | 2×1 | 中心差分 | 2m = 4 次 `StageCost` |
| lxx (Hessian) | 4×4 | 四点差分 | 4n² = 64 次 `StageCost` |
| luu (Hessian) | 2×2 | 四点差分 | 4m² = 16 次 `StageCost` |
| lux (交叉) | 2×4 | 四点差分 | 4nm = 32 次 `StageCost` |

代码位置：`src/ilqr/ilqr_solver.cpp` → `CentralDifferenceGradient*`, `CentralDifferenceHessian*`

### 2.3 约束（通过 AL 罚项间接求导）

约束被 `AugmentedLagrangianKnotCost` 包装后融入总代价，iLQR 对总代价差分时隐式覆盖了约束的导数。

#### 2.3.1 控制量边界约束（ControlBoxConstraint）

```
c(x,u) = [u_min - u;  u - u_max]    (不等式, 维度 2m = 4)
```

- 约束仅依赖 u，对 x 的偏导为零
- **适合解析求导**：梯度 ∂c/∂u = [-I; I]，Hessian 为零

#### 2.3.2 速度限制约束（SpeedLimitConstraint）

```
c(x,u) = [v_min - v;  v - v_max]    (不等式, 维度 2)
```

- 仅依赖 x(3)，非常简单
- **适合解析求导**：∂c/∂x 仅在第 3 列有 ±1

#### 2.3.3 道路边界约束 — 点模型（RoadBoundaryConstraint）

```
c(x,u) = [d_lb - d(x);  d(x) - d_ub]    (不等式, 维度 2)
```

- d(x) 是参考线横向偏差，对于直线参考线是 x(1) 的线性函数
- **适合解析求导**

#### 2.3.4 道路边界约束 — 多圆模型（MultiCircleRoadBoundaryConstraint）

```
对每个车身覆盖圆 j:
  c_{2j}   = (d_lb + r_j) - lateral(center_j(x))
  c_{2j+1} = lateral(center_j(x)) - (d_ub - r_j)
```

- 维度：2 × 圆数（通常 3 圆 → 6 维）
- center_j 通过刚体变换依赖 (x, y, θ)，包含 cos/sin
- **可以解析求导**，但需要手写刚体变换的 Jacobian

#### 2.3.5 圆形障碍物约束 — 点模型（CircularObstacleConstraint）

```
c(x,u) = r_safe² - [(x - cx)² + (y - cy)²]    (不等式, 维度 1)
```

- **适合解析求导**：∂c/∂x = [-2(x-cx), -2(y-cy), 0, 0]

#### 2.3.6 圆形障碍物约束 — 多圆模型（MultiCircleVehicleObstacleConstraint）

```
对每个车身覆盖圆 j:
  c_j = (r_obs + r_j)² - ‖center_j(x) - p_obs‖²
```

- 维度：圆数（通常 3）
- 需要对 center_j(x) 的刚体变换求导
- **可以解析求导**，链式法则 = 距离对圆心的导数 × 圆心对状态的 Jacobian

#### 2.3.7 多边形障碍物约束 — LSE 平滑（LSEPolygonObstacleConstraint）

```
对每个车身覆盖圆 j:
  c_j = r_j - LSE_α(center_j, polygon)
```

- LSE（Log-Sum-Exp）本身可微，是半平面距离的平滑近似
- **可以解析求导**，但公式较复杂（LSE 对圆心的梯度 + 刚体变换 Jacobian）

#### 2.3.8 多边形障碍物约束 — 精确距离（PolygonCollisionConstraint）

```
c(x,u) = safety_margin - dist(vehicle_polygon, obstacle_polygon)
```

- 依赖凸多边形对之间的最短距离 `ConvexPolygonPairDistance`
- **不易解析求导**：GJK / 最近点算法在顶点/边切换处不可微

#### 2.3.9 终端目标约束（TerminalGoalConstraint）

```
c(x_N) = x_N - x_target = 0    (等式, 维度 n)
```

- **适合解析求导**：∂c/∂x = I（单位矩阵）

---

## 3. 当前中心差分方法的函数调用量估算

设 n=4（状态维度）, m=2（控制维度）, H=50（时间步数）。

| 求导项 | 每步调用次数 | 总调用次数 (×H) |
|--------|-------------|----------------|
| A (∂f/∂x) | 2n = 8 次 NextState | 400 |
| B (∂f/∂u) | 2m = 4 次 NextState | 200 |
| lx | 2n = 8 次 StageCost | 400 |
| lu | 2m = 4 次 StageCost | 200 |
| lxx | 4n² = 64 次 StageCost | 3200 |
| luu | 4m² = 16 次 StageCost | 800 |
| lux | 4nm = 32 次 StageCost | 1600 |
| 终端 | 2n + 4n² = 72 | 72 |

**每次 iLQR 迭代**：~600 次 NextState + ~6200 次 StageCost（含 AL 罚项内部的约束 Evaluate）

注意：每次 `StageCost` 调用内部会遍历所有约束的 `Evaluate`。如果每步绑定 4 个约束，实际的约束函数调用量还要乘以 4。

---

## 4. 解析求导公式推导

以下对除 `PolygonCollisionConstraint` 外的所有组件给出解析导数公式。

状态定义 `x = [px, py, θ, v]ᵀ`，控制定义 `u = [a, δ]ᵀ`（加速度、前轮转角）。
参考线航向 `h`、原点 `(ox, oy)`、轮距 `L`。

### 4.1 KinematicBicycleModel — Jacobian A, B

动力学方程（前向欧拉）：

```
f₀ = px + dt·v·cos(θ)
f₁ = py + dt·v·sin(θ)
f₂ = θ  + dt·v·tan(δ)/L
f₃ = v  + dt·a
```

**A = ∂f/∂x（4×4）：**

```
     ∂px   ∂py   ∂θ                ∂v
f₀ [ 1     0    -dt·v·sin(θ)       dt·cos(θ)       ]
f₁ [ 0     1     dt·v·cos(θ)       dt·sin(θ)       ]
f₂ [ 0     0     1                 dt·tan(δ)/L      ]
f₃ [ 0     0     0                 1                ]
```

**B = ∂f/∂u（4×2）：**

```
     ∂a        ∂δ
f₀ [ 0         0                  ]
f₁ [ 0         0                  ]
f₂ [ 0         dt·v/(L·cos²(δ))   ]
f₃ [ dt        0                  ]
```

### 4.2 QuadraticCost

**Stage cost：** `ℓ = 0.5·(x-xr)ᵀQ(x-xr) + 0.5·(u-ur)ᵀR(u-ur)`

```
lx  = Q·(x - xr)
lu  = R·(u - ur)
lxx = Q
luu = R
lux = 0
```

**Terminal cost：** `ℓf = 0.5·(x-xr)ᵀQf(x-xr)`

```
lx  = Qf·(x - xr)
lxx = Qf
```

### 4.3 LaneTrackingCost

定义中间变量：

```
e_lat = -sin(h)·(px-ox) + cos(h)·(py-oy)    （横向偏差）
e_hdg = θ - h                                （航向偏差，归一化到[-π,π]）
e_spd = v - v_target                         （速度偏差）
```

对状态的 Jacobian：

```
∂e_lat/∂x = [-sin(h),  cos(h),  0,  0]
∂e_hdg/∂x = [0,        0,       1,  0]
∂e_spd/∂x = [0,        0,       0,  1]
```

**Stage cost：** `ℓ = 0.5·wl·e_lat² + 0.5·wh·e_hdg² + 0.5·ws·e_spd² + 0.5·wa·a² + 0.5·wd·δ²`

```
lx  = wl·e_lat·∂e_lat/∂x + wh·e_hdg·∂e_hdg/∂x + ws·e_spd·∂e_spd/∂x
lu  = [wa·a,  wd·δ]
lxx = wl·(∂e_lat/∂x)ᵀ(∂e_lat/∂x) + wh·(∂e_hdg/∂x)ᵀ(∂e_hdg/∂x) + ws·(∂e_spd/∂x)ᵀ(∂e_spd/∂x)
luu = diag(wa, wd)
lux = 0
```

即 `lxx` 为三个秩1矩阵之和（每个是外积加权）。

**Terminal cost：** 增加纵向偏差项：

```
e_lon = cos(h)·(px-ox) + sin(h)·(py-oy) - s_target
∂e_lon/∂x = [cos(h),  sin(h),  0,  0]

lx  = wl_t·e_lon·∂e_lon/∂x + wlat_t·e_lat·∂e_lat/∂x + wh_t·e_hdg·∂e_hdg/∂x + ws_t·e_spd·∂e_spd/∂x
lxx = wl_t·(∂e_lon/∂x)ᵀ(∂e_lon/∂x) + wlat_t·(∂e_lat/∂x)ᵀ(∂e_lat/∂x)
    + wh_t·(∂e_hdg/∂x)ᵀ(∂e_hdg/∂x) + ws_t·(∂e_spd/∂x)ᵀ(∂e_spd/∂x)
```

### 4.4 GuidanceTrackingCost

**Stage cost（非 terminal_mode）：**

```
ℓ = 0.5·wp·(dx² + dy²) + 0.5·wh·e_hdg² + 0.5·ws·e_spd²
```

其中 `dx = px - rx, dy = py - ry, e_hdg = NormalizeAngle(θ - r_yaw), e_spd = v - r_speed`

```
lx  = [wp·dx,  wp·dy,  wh·e_hdg,  ws·e_spd]
lu  = [0, 0]
lxx = diag(wp, wp, wh, ws)
luu = 0
lux = 0
```

**Terminal cost（terminal_longitudinal_only 模式）：**

```
e_lon = cos(h)·(px - rx) + sin(h)·(py - ry)
∂e_lon/∂x = [cos(h),  sin(h),  0,  0]

lx  = wp·e_lon·∂e_lon/∂x
lxx = wp·(∂e_lon/∂x)ᵀ(∂e_lon/∂x)
```

### 4.5 ControlRateCost

```
ℓ = 0.5·wj·(a - a_prev)² + 0.5·wsr·(δ - δ_prev)²
```

```
lx  = 0
lu  = [wj·(a - a_prev),  wsr·(δ - δ_prev)]
lxx = 0
luu = diag(wj, wsr)
lux = 0
```

### 4.6 ControlBoxConstraint — Jacobian

```
c = [u_lb - u;  u - u_ub]    (2m × 1)

∂c/∂x = 0                    (2m × n)
∂c/∂u = [-I;  I]             (2m × m)
```

### 4.7 SpeedLimitConstraint — Jacobian

```
c = [v_min - v;  v - v_max]  (2 × 1)

∂c/∂x = [ 0  0  0  -1 ]     (2 × 4)
         [ 0  0  0   1 ]
∂c/∂u = 0                    (2 × 2)
```

### 4.8 TerminalGoalConstraint — Jacobian

```
c = x - x_target             (n × 1)

∂c/∂x = I                    (n × n)
∂c/∂u 不适用（终端无控制）
```

### 4.9 RoadBoundaryConstraint — Jacobian

```
e_lat = -sin(h)·(px-ox) + cos(h)·(py-oy)
c = [d_lb - e_lat;  e_lat - d_ub]    (2 × 1)

∂e_lat/∂x = [-sin(h),  cos(h),  0,  0]

∂c/∂x = [ sin(h)   -cos(h)   0   0 ]    (2 × 4)
         [-sin(h)    cos(h)   0   0 ]
∂c/∂u = 0                                (2 × 2)
```

### 4.10 CircularObstacleConstraint — Jacobian

```
dx = px - cx,  dy = py - cy
c = r² - (dx² + dy²)         (1 × 1)

∂c/∂x = [-2dx,  -2dy,  0,  0]    (1 × 4)
∂c/∂u = 0                         (1 × 2)
```

### 4.11 MultiCircleRoadBoundaryConstraint — Jacobian

车身覆盖圆 j 的体坐标偏移 `(bx_j, 0)`（`by=0` 因为圆心在纵轴上）。

**刚体变换及其 Jacobian（公共子式）：**

```
center_j = [px + cos(θ)·bx_j,  py + sin(θ)·bx_j]

∂center_j_x/∂x = [1,  0,  -sin(θ)·bx_j,  0]
∂center_j_y/∂x = [0,  1,   cos(θ)·bx_j,  0]
```

横向偏差（参考线坐标系下）：

```
lat_j = -sin(h)·(center_j_x - ox) + cos(h)·(center_j_y - oy)

∂lat_j/∂x = -sin(h)·∂center_j_x/∂x + cos(h)·∂center_j_y/∂x
           = [-sin(h),  cos(h),  bx_j·cos(θ - h),  0]
```

约束输出：

```
c_{2j}   = (d_lb + r_j) - lat_j
c_{2j+1} = lat_j - (d_ub - r_j)

∂c_{2j}/∂x   = -∂lat_j/∂x = [sin(h),  -cos(h),  -bx_j·cos(θ-h),  0]
∂c_{2j+1}/∂x =  ∂lat_j/∂x = [-sin(h),  cos(h),   bx_j·cos(θ-h),  0]
∂c/∂u = 0
```

### 4.12 MultiCircleVehicleObstacleConstraint — Jacobian

```
center_j = [px + cos(θ)·bx_j,  py + sin(θ)·bx_j]
dx_j = center_j_x - obs_cx
dy_j = center_j_y - obs_cy
c_j = (r_obs + r_j)² - (dx_j² + dy_j²)

∂c_j/∂x = -2·dx_j·∂center_j_x/∂x - 2·dy_j·∂center_j_y/∂x
         = [-2dx_j,  -2dy_j,  2·bx_j·(dx_j·sin(θ) - dy_j·cos(θ)),  0]
∂c_j/∂u = 0
```

### 4.13 LSEPolygonObstacleConstraint — Jacobian

**LSE 函数对点 p 的梯度：**

```
LSE(p) = max_val + log(Σᵢ exp(α·(hᵢ - max_val))) / α

其中 hᵢ = nᵢᵀ·p - bᵢ,  max_val = maxᵢ(hᵢ)
```

softmax 权重：

```
wᵢ = exp(α·(hᵢ - max_val)) / Σⱼ exp(α·(hⱼ - max_val))

∂LSE/∂p = Σᵢ wᵢ · nᵢ    （法向量的加权平均）
```

约束输出：

```
c_j = r_j - LSE(center_j)

∂c_j/∂p = -∂LSE/∂p = -Σᵢ wᵢ · nᵢ

∂c_j/∂x = (∂c_j/∂p)ᵀ · ∂center_j/∂x    (1×4)

其中 ∂center_j/∂x = [1  0  -sin(θ)·bx_j  0]
                      [0  1   cos(θ)·bx_j  0]
∂c_j/∂u = 0
```

### 4.14 增广拉格朗日（AL）罚项展开公式

给定约束 `c(x,u)`（p 维向量），其 Jacobian 为 `Jx = ∂c/∂x (p×n)`，`Ju = ∂c/∂u (p×m)`。

**等式约束 AL 项：** `φ = Σⱼ [λⱼcⱼ + μⱼ/2·cⱼ²]`

```
∂φ/∂x = Jxᵀ · (λ + μ⊙c)
∂φ/∂u = Juᵀ · (λ + μ⊙c)
∂²φ/∂x² ≈ Jxᵀ · diag(μ) · Jx        (Gauss-Newton 近似)
∂²φ/∂u² ≈ Juᵀ · diag(μ) · Ju
∂²φ/∂u∂x ≈ Juᵀ · diag(μ) · Jx
```

**不等式约束 AL 项：** `φ = Σⱼ [max(0, λⱼ+μⱼcⱼ)² - λⱼ²] / (2μⱼ)`

定义活跃指示 `Iⱼ = 1 if (λⱼ + μⱼcⱼ) > 0, else 0`
定义投影值  `pⱼ = max(0, λⱼ + μⱼcⱼ)`

```
∂φ/∂x = Jxᵀ · (I ⊙ p)
∂φ/∂u = Juᵀ · (I ⊙ p)
∂²φ/∂x² ≈ Jxᵀ · diag(I ⊙ μ) · Jx    (Gauss-Newton 近似)
∂²φ/∂u² ≈ Juᵀ · diag(I ⊙ μ) · Ju
∂²φ/∂u∂x ≈ Juᵀ · diag(I ⊙ μ) · Jx
```

**总的 AL 代价展开 = 基础代价展开 + Σ 各约束 AL 项展开**

---

## 5. 可选的更优求导方式

**原理**：手动推导并编码每个函数的 Jacobian / Hessian 解析表达式。

**适用对象与优先级**：

| 优先级 | 组件 | 难度 | 收益 | 说明 |
|--------|------|------|------|------|
| ★★★ | KinematicBicycleModel A, B | 低 | 高 | 仅含 sin/cos/tan，4×4 和 4×2 矩阵，公式简洁 |
| ★★★ | QuadraticCost / LaneTrackingCost | 低 | 高 | 二次型代价，梯度和 Hessian 有闭式解 |
| ★★☆ | ControlBoxConstraint | 极低 | 中 | Jacobian 为常数矩阵 ±I |
| ★★☆ | SpeedLimitConstraint | 极低 | 中 | Jacobian 仅一个非零元素 |
| ★★☆ | CircularObstacleConstraint | 低 | 中 | Jacobian = [-2Δx, -2Δy, 0, 0] |
| ★★☆ | TerminalGoalConstraint | 极低 | 低 | Jacobian = I |
| ★☆☆ | MultiCircle 系列 | 中 | 中 | 需要推导刚体变换 Jacobian |
| ★☆☆ | LSEPolygonObstacleConstraint | 中高 | 中 | 需推导 LSE 梯度 + 刚体变换链式法则 |
| ☆☆☆ | PolygonCollisionConstraint | 高 | 低 | GJK 距离在顶点切换处不可微，不建议解析 |

**KinematicBicycleModel 解析 Jacobian 示例**：

```
状态 x = [x, y, θ, v]ᵀ,  控制 u = [a, δ]ᵀ

A = ∂f/∂x =
┌ 1    0    -dt·v·sinθ    dt·cosθ        ┐
│ 0    1     dt·v·cosθ    dt·sinθ        │
│ 0    0     1            dt·tanδ/L      │
└ 0    0     0            1              ┘

B = ∂f/∂u =
┌ 0              0                       ┐
│ 0              0                       │
│ 0              dt·v/(L·cos²δ)          │
└ dt             0                       ┘
```

**优点**：
- 精度最高（无截断误差）
- 速度最快（无循环差分）
- 对动力学 Jacobian：从 12 次 NextState 降到 0 次

**缺点**：
- 需为每个函数手写推导，维护成本高
- 容易出错（特别是复杂约束的链式法则）

---

### 4.2 自动微分（Automatic Differentiation）

**原理**：在编译期或运行期自动追踪计算图，通过链式法则精确求导。

**C++ 常用库**：

| 库 | 模式 | 特点 |
|----|------|------|
| **CppAD** | 前向 + 反向 | 成熟稳定，支持高阶导数，ALTRO 论文推荐 |
| **ADOL-C** | 前向 + 反向 | 工业级，tape-based |
| **ceres::Jet** | 前向 | 轻量，仅 header，适合小规模 |
| **Enzyme** | 编译期 LLVM | 零开销，但需 LLVM 插件 |
| **autodiff (C++)** | 前向 + 反向 | 现代 C++17，header-only，与 Eigen 兼容 |

**接入方式（以 CppAD 为例）**：

```cpp
// 将 NextState 模板化，支持 CppAD::AD<double> 类型
template <typename Scalar>
VectorX<Scalar> NextState(const VectorX<Scalar>& state,
                          const VectorX<Scalar>& control, double dt) {
    VectorX<Scalar> next(4);
    next(0) = state(0) + dt * state(3) * cos(state(2));
    next(1) = state(1) + dt * state(3) * sin(state(2));
    next(2) = state(2) + dt * state(3) * tan(control(1)) / L;
    next(3) = state(3) + dt * control(0);
    return next;
}
// CppAD 自动生成 Jacobian
```

**优点**：
- 精度与解析求导相同（机器精度）
- 通用性强——新增约束/代价后无需手动推导
- 前向模式 AD 的计算量 ≈ O(m) 倍原始函数（m 为输入维度）

**缺点**：
- 需要引入外部依赖
- 部分库（tape-based）有运行时开销
- 需要将现有函数改为模板化或使用 AD 类型

---

### 4.3 解析求导 + 约束级中心差分（混合方案，推荐）

**原理**：对容易求解析导数的部分（动力学、简单约束、二次代价）使用解析求导，对复杂约束保留中心差分。

**关键改动**：修改 `ConstraintFunction` 接口，增加可选的 Jacobian 方法：

```cpp
class ConstraintFunction {
 public:
  // 现有接口
  virtual Vector Evaluate(const Vector& state, const Vector& control) const = 0;

  // 新增（可选）：解析 Jacobian
  virtual bool HasAnalyticalJacobian() const { return false; }
  virtual Matrix JacobianState(const Vector& state, const Vector& control) const {
    throw std::logic_error("Not implemented");
  }
  virtual Matrix JacobianControl(const Vector& state, const Vector& control) const {
    throw std::logic_error("Not implemented");
  }
};
```

AL 罚项在展开时可检查 `HasAnalyticalJacobian()`，有则直接使用，否则回退到差分。

**同样地，`DynamicsModel` 也可增加可选的解析 Jacobian**：

```cpp
class DynamicsModel {
 public:
  virtual Vector NextState(...) const = 0;

  // 新增（可选）
  virtual bool HasAnalyticalJacobian() const { return false; }
  virtual Matrix JacobianState(const Vector& x, const Vector& u, double dt) const;
  virtual Matrix JacobianControl(const Vector& x, const Vector& u, double dt) const;
};
```

**优点**：
- 渐进式改进，不破坏现有代码
- 对简单组件获得精度和速度提升
- 复杂约束（如 PolygonCollision）保留差分兜底

**推荐实施顺序**：
1. KinematicBicycleModel 解析 Jacobian（收益最大）
2. QuadraticCost / LaneTrackingCost 解析梯度和 Hessian
3. ControlBoxConstraint、SpeedLimitConstraint 解析 Jacobian
4. CircularObstacleConstraint 解析 Jacobian
5. MultiCircle 系列解析 Jacobian
6. LSEPolygonObstacleConstraint 解析 Jacobian（如有精力）

---

### 4.4 约束 Jacobian 独立展开（跳过 AL 隐式差分）

**原理**：不再将约束融入标量代价后对标量差分，而是对约束值向量直接求 Jacobian，然后用链式法则组装 AL 项的梯度和 Hessian。

对于不等式约束 AL 项：

```
φ(x,u) = [max(0, λ + μ·c(x,u))² - λ²] / (2μ)
```

其梯度可分解为：

```
∂φ/∂x = max(0, λ + μ·c) · (∂c/∂x)      (当 λ + μ·c > 0 时)
∂²φ/∂x² = μ · (∂c/∂x)ᵀ(∂c/∂x) + max(0, λ + μ·c) · (∂²c/∂x²)
```

这样只需要约束的 Jacobian `∂c/∂x`（和可选的 Hessian），而不是对整个标量 AL 代价做全维度差分。

**优点**：
- 差分只在约束值向量上进行，维度通常远小于 n²
- 梯度精度显著提升（避免了通过标量代价间接差分的误差放大）

**缺点**：
- 需要重构 `AugmentedLagrangianKnotCost` 的展开逻辑
- 约束 Hessian 通常假设为零（Gauss-Newton 近似），引入近似误差

---

### 4.5 各方案对比总结

| 方案 | 精度 | 速度 | 实现难度 | 通用性 | 推荐度 |
|------|------|------|----------|--------|--------|
| 当前中心差分 | O(ε²) | 慢 | 已实现 | ★★★ | 基线 |
| 全部解析求导 | 机器精度 | 最快 | 高 | ★☆☆ | ★★☆ |
| 自动微分 (AD) | 机器精度 | 快 | 中（需重构） | ★★★ | ★★★ |
| 混合方案 | 机器精度/O(ε²) | 快 | 低（渐进式） | ★★★ | ★★★ |
| 约束 Jacobian 独立展开 | 高 | 中 | 中 | ★★☆ | ★★☆ |

**建议**：短期采用**混合方案（4.3）**，优先为动力学和二次代价提供解析导数；中长期考虑引入**自动微分（4.2）**实现全面精确求导。
