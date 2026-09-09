# Polygon `LSE` / `Exact` 问题记录

本文记录当前仓库中多边形障碍物约束的已知现象与代码级原因，目标是给后续排障、修复和方案取舍提供统一基线。

适用范围：

- `ObstacleType::kPolygonLSE`
- `ObstacleType::kPolygonExact`
- 当前前端入口 `apps/rerun_imgui_frontend.cpp`
- 当前 `AL-iLQR + finite-difference iLQR` 实现

## 1. 当前可观察现象

在当前实现下，三种障碍物模式的稳定性通常表现为：

- `Circle` 最稳定，最容易收敛，求解时间也通常最短。
- `PolygonLSE` 次之，能够工作，但比圆障碍物更敏感，容易出现收敛变慢、违反量下降不稳定、对初值更敏感。
- `PolygonExact` 最脆弱，最容易出现外层 AL 迭代推进缓慢、求解时间显著上升、最终残余违反量偏大或直接不收敛。

在前端中，问题更容易在以下组合下暴露：

- `Init Guess = Global Guidance` 或 `SL-DP`
- `use_guidance_tracking = true`
- `PP+iLQR Warmstart = ON`
- 障碍物类型切换到 `Polygon (LSE Smooth)` 或 `Polygon (Exact)`

典型表现包括：

- 切到 polygon 后，同一场景下比 circle 更容易失败。
- 轨迹初值在可视化上“看起来合理”，但最终 AL-iLQR 仍然无法把违反量压下去。
- `Exact` 模式下求解耗时明显增加，且不同初值之间结果波动更大。
- 打开 PP / GuidanceTracker warmstart 后，结果不一定更好，反而可能比默认初值更差。

## 2. 这不是“AL-iLQR 天生不能处理多边形”

需要明确区分两件事：

- 方法层面：`AL-iLQR` 并不只适用于圆障碍物，它可以处理一般不等式约束。
- 实现层面：当前仓库这版实现，对“光滑、导数稳定”的约束明显更友好，因此圆障碍物最稳，多边形约束尤其 `Exact` 更容易暴露数值问题。

换句话说，当前问题是“这套实现对 polygon 不够鲁棒”，不是“AL-iLQR 原理上不能做 polygon”。

## 3. 直接原因一：前端把 PP / GuidanceTracker 输出写成了最终求解器初值

当前前端在 `PreparePlanningScenario()` 中，如果存在 `selected_guidance`，会：

1. 先用 `PurePursuitTracker::Track()` 生成控制量；
2. 再在 `use_pp_ilqr_warmstart` 打开时，用 `GuidanceTracker::Track()` 进一步改写；
3. 最终把该结果直接写回 `prepared.scenario.initial_controls`。

对应代码：

- `apps/rerun_imgui_frontend.cpp`
- `src/autodrive/pure_pursuit_tracker.cpp`
- `src/autodrive/guidance_tracker.cpp`

这意味着最终 AL-iLQR 并不是在优化“场景默认 seed”或“planner 原生 controls”，而是在优化一个经过 PP / tracking 改写后的控制序列。

这件事在 circle 下通常还能工作，但在 polygon 下会更容易失稳，因为：

- PP 更像几何跟踪器，不保证与最终碰撞约束模型一致；
- GuidanceTracker 优化的是 guidance 跟踪目标，不等价于最终约束问题；
- 最终 seed 可能带来较大的 steering 饱和、较大的侧向激励或较差的末端回正行为。

因此，当前前端中的 warmstart 不是“天然更好”，它只是“更贴 guidance 的一个初值”，而这个初值与 polygon 约束的数值性质并不匹配。

## 4. 直接原因二：AL 不等式项在约束未激活时不会持续提供拉开障碍物的梯度

当前不等式增广项实现位于：

- `src/al/augmented_lagrangian_cost.cpp`

对不等式约束使用的是标准投影型 AL 项：

`max(0, lambda + mu * c(x, u))`

这带来一个重要性质：

- 当约束还没有“激活”时，增广项可能几乎不提供有效梯度；
- 只有当违反量变大、或者乘子 / 罚参数累积到一定程度时，障碍物项才会真正强势介入。

这对 circle 影响较小，因为 circle 约束本身平滑、方向明确、梯度更稳定。
但对 polygon 尤其 `Exact`，问题会被放大：

- 如果初值方向不对，AL 早期可能主要受基础 cost 和跟踪项主导；
- 等到真正靠近障碍物、约束开始强激活时，局部几何已经变坏，后续线性化更难救回来。

## 5. `LSE` 模式的特定问题

`LSEPolygonObstacleConstraint` 位于：

- `src/autodrive/lse_polygon_obstacle_constraint.cpp`
- `src/autodrive/convex_polygon.cpp`

其基本形式是：对多边形各面距离做 `LogSumExp` 光滑最大值，然后用车辆多圆与该值构造不等式约束。

当前实现的主要问题不是“不可微”，而是“光滑但不一定数值友好”：

- 它本质上仍然是“最大面距离”的光滑近似，而不是真实欧氏距离。
- 在顶点附近它天然保守，容易比真实所需间隙更紧。
- 当 `alpha` 取大时，会越来越接近 `max`，光滑性虽仍存在，但局部曲率会变陡，导致优化更敏感。
- 车辆几何使用的是多圆近似，而不是完整车身 polygon，这与 `Exact` 模式的几何含义不同，也与 guidance / tracking 阶段的行为不完全一致。

因此 `LSE` 的问题更像：

- 约束是光滑的，但几何意义不是“真实 polygon 距离”；
- 在边角、窄缝和强避障场景中，保守性和陡峭曲率会同时出现；
- 一旦初值带着较大的横向动作冲向障碍物边角，优化器更容易出现下降困难。

## 6. `Exact` 模式的特定问题

`PolygonCollisionConstraint` 和底层距离计算位于：

- `src/autodrive/polygon_collision_constraint.cpp`
- `src/autodrive/convex_polygon.cpp`

当前 `Exact` 使用的是：

- 分离时：顶点到边的精确欧氏最小距离；
- 重叠时：基于 SAT 的最小 overlap，返回负穿透深度。

这套定义在几何上是合理的，但对当前求解器不友好，原因是：

- 最近特征会切换：可能从“某个顶点到某条边最近”切到另一组特征。
- 分离 / 接触 / 穿透三个区域的局部表达不同。
- SAT overlap 和 vertex-edge 最小距离拼接后，函数通常只有分段光滑，而不是全局光滑。

这对依赖局部二次近似的 iLQR 很不友好。更具体地说：

- 在最近特征切换点附近，梯度会突变；
- 在接触附近，小扰动可能让最近边、最近顶点或 penetration 轴发生切换；
- 这会让局部近似质量明显下降，forward pass 更容易拒绝步长，outer loop 也更容易卡住。

## 7. 放大问题的底层原因：iLQR 当前使用有限差分求导

当前 iLQR 的 cost / dynamics 展开依赖 central difference，位于：

- `src/ilqr/ilqr_solver.cpp`

这意味着：

- 对平滑约束，有限差分通常足够；
- 对分段光滑甚至特征切换明显的约束，有限差分会把不连续导数直接暴露出来；
- 一旦差分步长跨过几何特征切换边界，得到的梯度 / Hessian 近似会明显失真。

这也是为什么：

- `Circle` 约束最稳；
- `LSE` 还能勉强工作；
- `Exact` 最容易把当前 iLQR 的数值弱点全部放大出来。

## 8. 另一个结构性原因：不同阶段使用的几何模型并不一致

当前链路中至少存在三套不同几何：

- guidance / tracking 关注的是参考轨迹跟踪质量；
- `LSE` 使用“多圆车辆 vs polygon obstacle”；
- `Exact` 使用“整车 rectangle polygon vs polygon obstacle”。

这会带来两个后果：

- 前段生成的“好初值”，不一定是后段约束意义下的“好初值”；
- 初值越依赖 guidance/PP 的几何直觉，越可能在最终碰撞约束里表现很差。

因此，当前问题并不只是“约束公式难”，还有“初值来源与最终约束语义不一致”。

## 9. 当前阶段可以确认的结论

可以确认：

- 当前仓库对 circle 障碍物明显更鲁棒。
- `LSE` 的主要问题是保守性、曲率陡峭和多圆几何近似带来的数值敏感性。
- `Exact` 的主要问题是分段光滑、最近特征切换、SAT / vertex-edge 拼接后对 finite-difference iLQR 不友好。
- 前端把 PP / GuidanceTracker 输出直接作为最终 solver 初值，会放大 polygon 模式下的不稳定性。
- 当前问题属于“实现与数值策略不够稳”，不是“AL-iLQR 理论上不能做 polygon”。

不能直接下的结论：

- 不能说 `AL-iLQR` 只适用于圆障碍物。
- 不能说 guidance 一定比默认初值差。
- 不能说只要提高罚参数，就能自动解决 polygon 的收敛问题。

## 10. 后续修复方向

后续修复应优先按下面顺序推进：

1. 先把“最终 solver 用什么初值”与“PP / GuidanceTracker 仅用于可视化或中间评估”明确解耦，避免前端悄悄改写最终初值。
2. 为不同初值来源建立显式策略：默认 seed、planner controls、PP seed、GuidanceTracker seed，分别评估，不要混在一起。
3. 对 polygon 模式优先保证几何一致性，避免 guidance 阶段和最终约束阶段使用完全不同的碰撞语义。
4. 如果继续保留 `Exact` 作为主优化约束，应考虑解析梯度、连续化 / continuation 或其他更稳的数值策略；仅靠当前 finite-difference iLQR 很难稳定。
5. 在工程上更务实的做法是：主优化使用光滑 surrogate（circle / smooth polygon distance），`Exact polygon` 先作为验证或碰撞检查层。

## 11. 一句话总结

当前 `LSE` 和 `Exact` 的问题，本质上不是“多边形不能做”，而是“当前这版前端初值链路 + AL 激活方式 + finite-difference iLQR + polygon 距离性质”组合在一起后，对多边形约束不够鲁棒。
