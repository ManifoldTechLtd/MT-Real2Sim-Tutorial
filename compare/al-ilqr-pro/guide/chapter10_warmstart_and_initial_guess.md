# Chapter 10: 初始解策略 — PP Warmstart 与 Guidance Tracking Cost 分析

> **对应代码**：`pure_pursuit_tracker.hpp/cpp`, `guidance_tracker.hpp/cpp`, `rerun_imgui_frontend.cpp`  
> **目标**：理解为什么 AL-iLQR 需要初始解、PP 和 PP+iLQR 两种 warmstart 策略的设计思路、以及 GuidanceTrackingCost 为何被移除

---

## 10.1 为什么 AL-iLQR 需要初始解

### 问题本质

AL-iLQR 是一种**局部优化算法**。它从一个初始控制序列 $\{u_0, u_1, \ldots, u_{H-1}\}$ 出发，通过迭代线性化和二次近似逐步改进轨迹。这意味着：

1. **算法只能找到初始点附近的局部最优解**，无法跳出初始邻域去探索全局
2. **如果初始解不在可行域附近**，优化器可能发散或收敛到低质量的局部最优
3. **如果初始解违反约束太严重**，增广拉格朗日的惩罚项会产生巨大梯度，导致数值不稳定

### 一个直观的类比

想象在一座有多个山谷的地形上放一个球：

- **全零初始解**（`u_k = 0`）：球被放在山顶，它可能滚向任意方向 — 最终结果不可预测
- **好的初始解**：球被放在正确山谷的斜坡上 — 它自然滚向最优解
- **差的初始解**：球被放在错误的山谷 — 它滚向错误的局部最优

### 初始解的两个核心要求

| 要求 | 说明 |
|------|------|
| **动力学可行** | 控制量经动力学模型 rollout 后，产生的轨迹在物理上是合理的（不穿墙、不瞬移） |
| **拓扑正确** | 轨迹在障碍物的正确一侧（从左侧绕行 vs 右侧绕行），这决定了优化器收敛到哪个局部最优 |

---

## 10.2 系统架构：三阶段规划流水线

本系统采用三阶段流水线来生成最终轨迹：

```
┌─────────────────┐     ┌──────────────────┐     ┌──────────────────────┐
│  Stage 1:       │     │  Stage 2:        │     │  Stage 3:            │
│  全局规划器      │ ──→ │  Warmstart 生成  │ ──→ │  避障 AL-iLQR 求解   │
│  (Guidance/DP)  │     │  (PP / PP+iLQR)  │     │  (最终轨迹)          │
└─────────────────┘     └──────────────────┘     └──────────────────────┘
     决策层                  动力学适配层               优化层
   输出: 路径点            输出: 控制量序列           输出: 最优轨迹
   (拓扑决策)             (动力学可行)              (满足所有约束)
```

每个阶段有明确的职责边界，不互相越界。

---

## 10.3 Stage 2 方案一：Pure Pursuit Warmstart（PP-only）

### 算法原理

Pure Pursuit（纯追踪）是经典的路径跟踪算法。核心思想：

1. **找最近点**：在 guidance 路径上找到距离当前车辆位置最近的点
2. **预瞄**：从最近点沿路径前方走一段距离（look-ahead distance），找到预瞄目标点
3. **转向**：计算驶向预瞄目标点所需的转向角

对于自行车模型，转向角公式为：

$$\delta = \arctan\left(\frac{2 L \sin\alpha}{L_d}\right)$$

其中 $L$ 是轴距，$\alpha$ 是预瞄点相对于车头方向的夹角，$L_d$ 是到预瞄点的欧氏距离。

### 关键工程细节

#### 弧长距离搜索（而非索引搜索）

输入的 guidance 点密度可能不均匀。如果用固定索引窗口（如 `last_near + 5`），在密集区域搜索范围太小，在稀疏区域又太大。正确做法是用弧长距离控制搜索范围：

```cpp
double max_s = g[last_near].s + std::max(1.0, v_est * dt * 5.0);
for (int j = last_near; j < gs; ++j) {
    if (g[j].s > max_s) break;  // 弧长距离限制，不会跳到路径的其他段
    // ... 找最近点
}
```

这对 S 弯、U 弯、自交叉路径至关重要 — 防止最近点搜索跳到路径的另一段。

#### 曲率自适应预瞄距离

固定预瞄距离在直道和弯道上表现不同。在急弯处需要更短的预瞄来紧贴路径：

$$L_d = \frac{L_{d,base}}{1 + \kappa \cdot L_{d,base} \cdot 3}$$

其中 $\kappa$ 是最近点处的局部曲率。效果：
- 直道（$\kappa \approx 0$）：$L_d \approx L_{d,base}$（稳定）
- R=2m 弯（$\kappa = 0.5$）：$L_d$ 显著缩短（紧贴）

#### 多模型支持

PP 模块通过 `state_dim` + `control_dim` + `wheelbase` 自动识别模型类型：

| 模型 | 状态维度 | 转向公式 |
|------|---------|---------|
| PointMass 2D | 4 (x,y,vx,vy) | PD 控制朝预瞄方向 |
| Unicycle | 3 (x,y,θ) | $\omega = 2v\sin\alpha / L_d$ |
| Bicycle | 4 (x,y,θ,v) | $\delta = \arctan(2L\sin\alpha / L_d)$ |

### 接口

```cpp
#include "autodrive/pure_pursuit_tracker.hpp"

auto result = PurePursuitTracker::Track(
    guidance, x0, dynamics, u_lb, u_ub, horizon, dt, wheelbase);
// result.controls → 可直接作为 AL-iLQR 初始控制量
// result.states   → 闭环 rollout 轨迹
// result.max_pos_error → 最大跟踪误差
```

### 性能

- 耗时 < 1ms
- 典型跟踪误差 < 0.5m（与路径曲率和模型能力有关）

---

## 10.4 Stage 2 方案二：PP + 无障碍 AL-iLQR Warmstart

### 动机

PP 是纯反馈控制器，不考虑未来。在某些复杂路径上（急弯连续、速度变化大），PP 的跟踪误差可能达到 1-2m。如果避障 AL-iLQR 从一个偏差较大的初始解开始，可能需要更多迭代才能收敛。

PP+iLQR warmstart 的思路：用 PP 生成一个粗糙但合理的初始控制序列，然后用一次**无障碍的 AL-iLQR 求解**来精化它。

### 流程

```
guidance → PP rollout → PP controls 作为初始解 → 无障碍 AL-iLQR 求解 → 精化后的 controls
                                                    ↑
                                          只有控制量/速度/道路约束
                                          不加障碍物约束
```

### 接口

```cpp
#include "autodrive/guidance_tracker.hpp"

GuidanceTracker tracker(vehicle_config, road_lb, road_ub, vehicle_geometry);
auto result = tracker.Track(x0, guidance, {}, horizon, dt, config);
// result.controls → 动力学精确可行的控制序列
```

### 何时使用

| 场景 | 推荐方案 |
|------|---------|
| 简单场景、低延迟要求 | PP-only（<1ms） |
| 复杂路径、高精度需求 | PP+iLQR（~5-15ms） |

### UI 控制

UI 面板上的 **"PP+iLQR Warmstart"** 复选框控制：
- **关闭**（默认）：仅用 PP → 直接给避障 AL-iLQR
- **开启**：PP → 无障碍 AL-iLQR → 避障 AL-iLQR

---

## 10.5 GuidanceTrackingCost：分析与结论

### 它是什么

`GuidanceTrackingCost` 是一个在**避障 AL-iLQR 的代价函数**中添加的额外项，作用是惩罚最终轨迹与 guidance 路径之间的偏差：

$$J_{tracking} = \sum_{k=0}^{H} \left[ w_{pos} \|p_k - p_k^{ref}\|^2 + w_{hdg} (\theta_k - \theta_k^{ref})^2 + w_{spd} (v_k - v_k^{ref})^2 \right]$$

它与参考线跟踪代价（`LaneTrackingCost`）是同级别的优化目标。

### 为什么被移除

经过分析和实践，`GuidanceTrackingCost` 存在以下问题：

#### 1. 职责混淆

全局规划器的作用是**做拓扑决策**（从障碍物左边走还是右边走）。这个信息应该通过**初始解**传递给优化器，让优化器在正确的邻域内搜索。把 guidance 强加到代价函数中，等于把"建议"变成了"命令"。

```
                正确做法                          错误做法
┌────────────┐              ┌────────────┐
│ 全局规划器  │              │ 全局规划器  │
│ → 拓扑决策  │              │ → 拓扑决策  │
└─────┬──────┘              └──┬─────┬───┘
      │ 通过初始解传递            │     │ 通过 cost 强加
      ↓                        ↓     ↓
┌─────────────┐          ┌──────────────┐
│ AL-iLQR     │          │ AL-iLQR      │
│ 自由优化     │          │ 被 guidance  │
│ (自己的cost) │          │ cost 束缚    │
└─────────────┘          └──────────────┘
```

#### 2. 目标冲突

避障 AL-iLQR 已经有自己的代价体系：

| 代价项 | 目标 |
|--------|------|
| 参考线跟踪（`LaneTrackingCost`） | 贴近车道中心线 |
| 速度目标 | 达到目标速度 |
| 控制量正则化 | 舒适性（平滑加速、转向） |
| 障碍物约束 | 不碰撞 |

加入 `GuidanceTrackingCost` 后，优化器同时被两条不同的参考线拉扯（车道中心线 vs guidance 路径），产生矛盾。特别是在避障场景中，guidance 说"向左偏"，但参考线 cost 说"回到中心"，两者打架导致优化结果不理想。

#### 3. 有了好的初始解就不需要

当初引入 `GuidanceTrackingCost` 的一个动机是：初始解质量差，需要用 cost 来"引导"优化器。现在有了 PP warmstart，初始轨迹已经在 guidance 附近（误差 < 0.5m），优化器自然会在这个邻域内搜索。guidance 的拓扑信息已经通过初始解隐式传递，不需要在 cost 中重复。

#### 4. 过度约束

`GuidanceTrackingCost` 的权重如果设大，会限制优化器的自由度，阻止它找到比 guidance 更好的解。如果设小，又形同虚设。不存在一个"恰好合适"的权重。

### 结论

> **`GuidanceTrackingCost` 已被默认关闭。** 全局规划器的拓扑决策通过 PP warmstart 以初始解的形式传递给避障 AL-iLQR，而非通过代价函数强加。这使得各模块职责清晰、互不越界。

设计原则：

- **全局规划器**：负责拓扑决策（走哪条路）
- **PP / PP+iLQR**：负责将拓扑决策转化为动力学可行的初始控制量
- **避障 AL-iLQR**：负责在初始解的邻域内做约束优化，使用自己的代价体系

---

## 10.6 代码结构总结

```
include/autodrive/
├── pure_pursuit_tracker.hpp    ← PP 模块接口
├── guidance_tracker.hpp        ← PP+iLQR 模块接口
└── guidance_tracking_cost.hpp  ← GuidanceTrackingCost (保留代码, 默认不启用)

src/autodrive/
├── pure_pursuit_tracker.cpp    ← PP 实现 (Query + Track)
└── guidance_tracker.cpp        ← GuidanceTracker 实现 (内部调用 PP)
```

调用关系：

```
rerun_imgui_frontend.cpp
  │
  ├── PurePursuitTracker::Track(guidance, x0, ...)   ← 始终执行
  │     └── 输出: initial_controls (PP)
  │
  ├── [可选] GuidanceTracker::Track(x0, guidance, pp_controls, ...)
  │     ├── 内部: 用 pp_controls 作为 AL-iLQR warmstart
  │     └── 输出: initial_controls (精化后)
  │
  └── ALILQRSolver::Solve(x0, initial_controls)   ← 避障最终求解
        └── 输出: 最终轨迹
```
