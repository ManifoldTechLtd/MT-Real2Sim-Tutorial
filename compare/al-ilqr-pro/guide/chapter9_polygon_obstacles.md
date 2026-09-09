# Chapter 9：多边形障碍物碰撞约束

## 9.1 背景：为什么需要多边形障碍物

在前面的章节中，我们的障碍物都是**圆形**的。圆形障碍物的碰撞检测简单——两个圆心之间的距离大于两个半径之和即可。但真实世界中，障碍物通常不是圆形的：停靠车辆是矩形的、交通锥是三角形的、护栏是长条形的。

**圆形近似的局限性**：用圆去包围一个矩形障碍物，必须使用外接圆（或多个小圆），前者极度保守（浪费大量可通行空间），后者本质上又变成了多圆-多圆检测问题。

本章介绍两种面向 AL-iLQR 框架的凸多边形障碍物处理方法：

| 方法 | 车辆表示 | 障碍物表示 | 距离计算 | 光滑性 |
|------|---------|-----------|---------|--------|
| **方法 A：LSE 光滑距离** | 多圆覆盖 | 凸多边形 (H-表示) | Log-Sum-Exp | C∞ |
| **方法 B：精确碰撞检测** | 矩形多边形 | 凸多边形 | 顶点-边枚举 + SAT | C⁰（分段光滑） |

---

## 9.2 基础数据结构：ConvexPolygon

### 9.2.1 凸多边形的 H-表示

一个凸多边形可以用顶点列表定义，也可以等价地用**半平面交集**（H-representation）定义：

$$\text{Polygon} = \{ \mathbf{p} \in \mathbb{R}^2 \mid \mathbf{n}_i^\top \mathbf{p} \leq b_i, \quad i = 1, \ldots, m \}$$

其中 $\mathbf{n}_i$ 是第 $i$ 条边的**单位外法向量**，$b_i$ 是对应偏移量。

**几何含义**：$\mathbf{n}_i^\top \mathbf{p} - b_i$ 表示点 $\mathbf{p}$ 到第 $i$ 条面的**带符号距离**：
- $> 0$：点在面外侧（多边形外部方向）
- $< 0$：点在面内侧（多边形内部方向）
- $= 0$：点在面上

### 9.2.2 从顶点计算 H-表示

给定逆时针（CCW）排列的顶点 $\mathbf{v}_0, \mathbf{v}_1, \ldots, \mathbf{v}_{m-1}$：

1. 第 $i$ 条边方向：$\mathbf{e}_i = \mathbf{v}_{(i+1) \% m} - \mathbf{v}_i$
2. 外法向量：$\mathbf{n}_i = (e_{i,y},\ -e_{i,x}) / \| \cdot \|$
3. 偏移量：$b_i = \mathbf{n}_i^\top \mathbf{v}_i$

对应代码位于 `ConvexPolygon` 构造函数：

```cpp
// src/autodrive/convex_polygon.cpp
for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    Vector edge = vertices_[j] - vertices_[i];
    Vector normal(2);
    normal(0) = edge(1);     // dy  →  outward x
    normal(1) = -edge(0);    // -dx →  outward y
    double len = normal.norm();
    normal /= len;
    outward_normals_.push_back(normal);
    offsets_.push_back(normal.dot(vertices_[i]));
}
```

### 9.2.3 辅助工厂函数

项目提供了方便的工厂函数来创建常见的多边形：

- `MakeRectangle(cx, cy, half_length, half_width, yaw)` — 以 `(cx, cy)` 为中心、给定半长/半宽和偏航角的矩形
- `MakeVehiclePolygon(x, y, yaw, length, width, rear_axle_to_rear)` — 根据车辆状态 `(x, y, yaw)` 和车体尺寸生成车辆矩形多边形，参考点为后轴中心

---

## 9.3 方法 A：Log-Sum-Exp 光滑距离（LSE）

### 9.3.1 问题

**点在多边形外部** 等价于：

$$\max_{i} \left( \mathbf{n}_i^\top \mathbf{p} - b_i \right) > 0$$

但 `max` 函数在两个面距离相等时**不可微**（存在折点），直接用于 iLQR 的二阶近似会出问题。

### 9.3.2 Log-Sum-Exp 光滑化

用 Log-Sum-Exp (LSE) 近似 `max`：

$$\text{LSE}_\alpha(\mathbf{p}) = \frac{1}{\alpha} \ln \left( \sum_{i=1}^{m} \exp\left( \alpha \left( \mathbf{n}_i^\top \mathbf{p} - b_i \right) \right) \right)$$

**关键性质**：

| 性质 | 说明 |
|------|------|
| 光滑性 | $C^\infty$，对任意有限 $\alpha$ 无限次可微 |
| 逼近精度 | $\max \leq \text{LSE}_\alpha \leq \max + \frac{\ln m}{\alpha}$ |
| 参数控制 | $\alpha \to \infty$ 时精确逼近 max；$\alpha$ 越小越光滑 |
| 单调性 | 外部为正，内部为负，与 `max` 符号一致 |

**数值稳定实现**：先提取 `max_val`，避免 `exp` 溢出：

```cpp
// src/autodrive/convex_polygon.cpp — ConvexPolygon::LogSumExpDistance
double max_val = -inf;
for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    max_val = std::max(max_val, val);
}
double sum_exp = 0.0;
for (int i = 0; i < FaceCount(); ++i) {
    double val = outward_normals_[i].dot(point) - offsets_[i];
    sum_exp += std::exp(alpha * (val - max_val));  // shift 保证不溢出
}
return max_val + std::log(sum_exp) / alpha;
```

### 9.3.3 约束构造

结合多圆车辆模型，对每个覆盖圆 $j$ 构造不等式约束：

$$c_j(\mathbf{x}) = r_j - \text{LSE}_\alpha(\mathbf{p}_j) \leq 0$$

其中 $\mathbf{p}_j$ 是第 $j$ 个覆盖圆在世界坐标系的圆心，$r_j$ 是其半径。

- 当 $\text{LSE} > r_j$ 时：圆心足够远离多边形，约束满足
- 当 $\text{LSE} < r_j$ 时：圆可能与多边形重叠，约束违反

```cpp
// src/autodrive/lse_polygon_obstacle_constraint.cpp
Vector LSEPolygonObstacleConstraint::Evaluate(...) const {
    const auto centers = vehicle_geometry_.EvaluateWorldCenters(state);
    const auto& circles = vehicle_geometry_.Circles();
    Vector values(OutputDim());
    for (int i = 0; i < OutputDim(); ++i) {
        const double lse = obstacle_polygon_.LogSumExpDistance(centers[i], alpha_);
        values(i) = circles[i].radius - lse;
    }
    return values;
}
```

### 9.3.4 α 参数选择

| $\alpha$ | 近似精度 | 光滑性 | 建议场景 |
|----------|---------|--------|---------|
| 5–10 | 较低 | 非常光滑 | 初始调试、快速收敛 |
| **15–25** | **适中** | **良好** | **一般场景（推荐）** |
| 40–100 | 很高 | 接近不光滑 | 精度要求高的场景 |

项目默认 $\alpha = 20$，在精度和光滑性之间取得良好平衡。

### 9.3.5 LSE 方法的保守性分析

在多边形**顶点**附近，LSE 方法存在轻微的保守性：

- 真实欧几里得距离 = 到顶点的距离（例如 $d\sqrt{2}$）
- LSE ≈ max(面距离) = $d$（面距离总是 ≤ 顶点距离）
- 约束要求 $d \geq r$，但实际只需 $d/\sqrt{2} \geq r$

这意味着 LSE 方法在顶点附近**比必要的更严格**，但从安全角度看这是有益的。

---

## 9.4 方法 B：精确碰撞检测（多边形-多边形距离）

### 9.4.1 动机

LSE 方法仍然需要用多圆近似车辆。方法 B 直接将车辆和障碍物都建模为**凸多边形**，计算它们之间的精确最小距离。

这种方法的灵感来源于 Tracy & Manchester 的论文 *"Differentiable Collision Detection for a Set of Convex Primitives"* (2023, ICRA)，其核心思想是：凸体之间的最小距离函数在大部分区域是可微的，可以直接用于基于梯度的轨迹优化。

### 9.4.2 带符号最小距离

两个凸多边形 $A$ 和 $B$ 之间的**带符号距离**：

$$d_{\text{signed}}(A, B) = \begin{cases} \min_{\mathbf{a} \in A, \mathbf{b} \in B} \|\mathbf{a} - \mathbf{b}\| & \text{分离时（正值）} \\ -\text{penetration\_depth}(A, B) & \text{重叠时（负值）} \end{cases}$$

### 9.4.3 分离检测：SAT（Separating Axis Theorem）

对两个凸多边形的所有面法向量，将两个多边形投影到该轴上：

```
轴方向 n_i:
    A 的投影: [min_a, max_a]
    B 的投影: [min_b, max_b]
    重叠量 = min(max_a, max_b) - max(min_a, min_b)
```

- 如果**任一**轴上重叠量 < 0 → 多边形分离
- 如果**所有**轴上重叠量 > 0 → 多边形重叠，最小重叠量 = 穿透深度

```cpp
// src/autodrive/convex_polygon.cpp — ConvexPolygonPairDistance
auto testAxes = [&](const ConvexPolygon& poly) {
    for (int i = 0; i < poly.FaceCount(); ++i) {
        const Vector& axis = poly.OutwardNormals()[i];
        // 投影 A 和 B 到 axis 上，计算重叠量
        double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
        if (overlap < 0.0) separated = true;
        min_overlap = std::min(min_overlap, overlap);
    }
};
testAxes(a);
testAxes(b);
```

### 9.4.4 分离时的精确距离

当多边形分离时，最小距离发生在**顶点-边**或**顶点-顶点**对上。通过枚举所有顶点-边组合计算：

$$d_{\min} = \min \left\{ \min_{v \in A} d(v, \partial B), \quad \min_{v \in B} d(v, \partial A) \right\}$$

其中 $d(v, \partial P)$ 是点到多边形边界的最小距离。

### 9.4.5 约束构造

每个时间步，根据当前状态构造车辆多边形，计算与障碍物多边形的距离：

```cpp
// src/autodrive/polygon_collision_constraint.cpp
Vector PolygonCollisionConstraint::Evaluate(...) const {
    ConvexPolygon vehicle = MakeVehiclePolygon(
        state(0), state(1), state(2),
        vehicle_length_, vehicle_width_, rear_axle_to_rear_);
    double dist = ConvexPolygonPairDistance(vehicle, obstacle_polygon_);
    Vector values(1);
    values(0) = safety_margin_ - dist;
    return values;
}
```

约束形式：$c(\mathbf{x}) = d_{\text{safe}} - d_{\text{signed}}(V(\mathbf{x}), O) \leq 0$

### 9.4.6 光滑性分析

精确距离函数的光滑性不如 LSE 方法：

- **面-面最近对**时：距离函数关于状态是线性的（C∞）
- **顶点-面最近对**时：距离是光滑的（C∞）
- **最近特征切换**时（如从面-面变为顶点-面）：距离函数连续但导数不连续（C⁰）

然而，项目使用**数值有限差分**求导，不依赖解析梯度。在特征切换点附近，有限差分仍能产生合理的梯度近似，实践中足以让 AL-iLQR 收敛。

### 9.4.7 与 Tracy & Manchester 论文的关系

本项目的方法 B 采用了论文的核心思想——直接计算凸体间最小距离用于轨迹优化约束——但实现上有简化：

| 方面 | 论文方法 | 本项目实现 |
|------|---------|-----------|
| 距离计算 | QP 求解（closest-point QP） | 顶点-边枚举 + SAT |
| 梯度获取 | KKT 条件隐式微分（解析） | 数值有限差分 |
| 支持形状 | 任意凸原语集合 | 凸多边形 |
| 穿透处理 | 专门的穿透距离公式 | SAT 穿透深度 |

---

## 9.5 两种方法对比

| 维度 | 方法 A (LSE) | 方法 B (精确碰撞) |
|------|-------------|------------------|
| **车辆建模** | 多圆覆盖 | 精确矩形多边形 |
| **障碍物建模** | 凸多边形 | 凸多边形 |
| **光滑性** | C∞（无限光滑） | C⁰（特征切换处不光滑） |
| **精度** | 轻微保守（顶点附近） | 精确 |
| **约束数量** | 每圆一个（2×障碍物数） | 每障碍物一个 |
| **计算成本** | O(圆数 × 面数) | O(顶点数²) |
| **收敛性** | 更稳定（因光滑） | 可能偶有振荡 |
| **推荐场景** | 通用场景，首选 | 需要精确碰撞的场景 |

### 选择建议

- **大多数场景**：优先使用**方法 A (LSE)**。C∞ 光滑性是 iLQR 二阶近似的理想保证。
- **精度敏感场景**（如自动泊车、窄通道）：考虑**方法 B**。车辆形状不再被多圆近似，空间利用更充分。
- **混合使用**：可以对不同障碍物使用不同方法——静态大障碍物用方法 B 精确处理，动态小障碍物用方法 A 快速光滑处理。

---

## 9.6 代码文件索引

| 文件 | 内容 |
|------|------|
| `include/autodrive/convex_polygon.hpp` | ConvexPolygon 类定义、MakeRectangle、MakeVehiclePolygon |
| `src/autodrive/convex_polygon.cpp` | H-表示构造、LSE 距离、精确 SDF、多边形对距离 |
| `include/autodrive/lse_polygon_obstacle_constraint.hpp` | LSE 约束类定义 |
| `src/autodrive/lse_polygon_obstacle_constraint.cpp` | LSE 约束实现 |
| `include/autodrive/polygon_collision_constraint.hpp` | 精确碰撞约束类定义 |
| `src/autodrive/polygon_collision_constraint.cpp` | 精确碰撞约束实现 |
| `tests/phase14_polygon_obstacle_test.cpp` | 单元测试 |

---

## 9.7 参考文献

1. Tracy, Manchester. *"Differentiable Collision Detection for a Set of Convex Primitives"*, ICRA 2023.
2. Boyd, Vandenberghe. *"Convex Optimization"*, Chapter 8 — 关于 Log-Sum-Exp 作为 max 的光滑近似。
3. Ericson. *"Real-Time Collision Detection"*, Chapter 5 — SAT 和顶点-边距离算法。
