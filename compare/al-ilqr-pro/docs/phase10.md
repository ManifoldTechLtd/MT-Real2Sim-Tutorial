# Phase 10

## 目标

在不重写整个求解器的前提下，补一层真正有数值收益的增强，让 `AL-iLQR` 在更激进的外层参数下也更稳。

这一阶段没有直接实现完整 `ALTRO` 的全部数值技巧，而是先选了两个最值得落地的增强：

- 更合理的 penalty update 参考策略
- best-so-far trajectory 保留策略

## 本阶段实现的功能

- `AL-iLQR` 外层 penalty 更新不再直接对比“上一轮 violation”
- penalty update 现在以“上次提高 penalty 时的 violation”作为参考
- 当 outer iteration 在相邻两轮之间明显停滞或回弹时，也会触发 penalty update
- `ALILQRSolver` 支持返回 best-so-far trajectory
- `ALILQROuterIterationLog` 增加：
  - `best_violation_so_far`
  - `penalty_updated`
- `ALILQRResult` 增加：
  - `final_violation`
  - `best_violation`
- 增加 `phase10_numerical_enhancement_demo`
- 增加 `phase10_numerical_enhancement_test`

## 对应理论

这一阶段主要参考 `ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf` 里的工程思想，而不是复刻某一条具体公式。

核心思路是：

1. 外层 penalty 不应该因为相邻两轮之间的小波动就频繁增长
2. 在非凸问题里，后续迭代不保证一定优于前面某一轮
3. 求解器应该保留“目前找到的最好约束满足程度轨迹”

因此这里的改动，本质上是在增强外层 augmented Lagrangian 的鲁棒性。

## 为什么这两个增强有效

### 1. penalty 参考点改成“上次涨 penalty 时的 violation”

旧逻辑是拿当前 violation 和上一轮 violation 比较。

这会有一个明显问题：

- 如果 violation 已经进入震荡区，哪怕只是轻微回弹，也可能不断触发 penalty 增长

新逻辑改成：

- 只有当当前 violation 相对“上次涨 penalty 时的参考 violation”下降不足，才继续涨 penalty

这更接近“penalty 分阶段推进”的思路，抖动会小很多。

另外，这一阶段还补了一条更贴近工程直觉的保护：

- 如果 penalty 还没涨，但 outer loop 已经进入“相邻两轮几乎不降或者开始回弹”的状态，也允许触发 penalty update

这样可以避免 solver 长时间卡在“约束仍然明显违反，但因为第一轮下降过大，所以后面永远不涨 penalty”的状态。

### 2. 返回 best-so-far trajectory

在非凸约束轨迹优化里，经常会出现这种情况：

- 中间某一轮已经找到更好的轨迹
- 后面因为 penalty 更强，或者局部线性化更差，最后一轮反而更坏

如果仍然机械地返回最后一轮结果，用户就会拿到一个更差的轨迹。

所以这里改成：

- 记录整个 outer loop 中 violation 最小的轨迹
- 默认返回这个 best-so-far 结果

这对教学和工程都更合理。

## 当前实现的边界

这一阶段仍然没有做：

- square-root backward pass
- active-set polishing
- slack variable
- 更深的 infeasible-start 机制

这些内容仍然属于后续可以继续扩展的方向。

## 验收标准

- 更激进的 outer-loop 参数下，solver 不会只返回最后一轮坏结果
- outer log 可以直接看见：
  - 哪一轮涨了 penalty
  - best violation 是否单调不升
- demo 和测试通过
