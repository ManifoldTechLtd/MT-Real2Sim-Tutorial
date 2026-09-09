# Phase 5

## 目标

在不引入增广拉格朗日之前，先把约束层单独建好。

这一阶段的关键词是：

- equality constraint
- inequality constraint
- stage constraint
- terminal constraint
- violation evaluation

## 本阶段实现的功能

- 增加通用 `ConstraintFunction` 接口
- 区分等式约束和不等式约束
- 增加 `ControlBoxConstraint`
- 增加 `TerminalGoalConstraint`
- 增加 `ConstrainedOptimalControlProblem`
- 支持沿整条轨迹评估约束值和最大违反量
- 提供 `phase5_constraint_evaluation_demo`

## 对应理论

本阶段对应的是约束轨迹优化问题的建模层，而不是求解层。

也就是说，我们先回答两个问题：

1. 一个约束在代码里怎么表达？
2. 一条轨迹的约束违反量怎么统计？

这对应 `AL-iLQR` 进入 augmented cost 之前必须准备好的基础设施。

## 当前实现的约定

- 等式约束：目标是 `h(x, u) = 0`
  违反量取 `|h|` 的无穷范数

- 不等式约束：目标是 `g(x, u) <= 0`
  违反量取 `max(g, 0)` 的无穷范数

## 为什么这一阶段还不直接把约束接进求解器

因为 `Phase 5` 的目标是把建模和评估独立验证清楚。

如果现在就把约束直接并入 `iLQR`：

- 很难区分是约束建模错了，还是 augmented Lagrangian 实现错了
- 也不利于教学

所以这里先停在“约束可表达、可统计、可验证”。

## 验收标准

- 可以在问题上挂 stage constraint 和 terminal constraint
- 可以对一条轨迹输出约束值和最大违反量
- 测试通过
