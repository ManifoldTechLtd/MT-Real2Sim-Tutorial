# Phase 6

## 目标

把 `Phase 5` 的约束对象真正并入代价函数，形成 augmented Lagrangian cost。

这一阶段仍然不做完整外循环，只做两件事：

- 固定 `lambda, mu`，构造一个无约束子问题
- 对这组 `lambda, mu` 实现 dual update 和 penalty update

## 本阶段实现的功能

- 增加 `AugmentedLagrangianKnotCost`
- 增加 `AugmentedLagrangianProblem`
- 把 stage/terminal constraints 包装成时变 AL cost
- 支持 dual variables 更新
- 支持 penalty parameters 更新
- 支持最大 penalty 和最大 violation 统计
- 提供 `phase6_augmented_lagrangian_demo`

## 对应理论

本阶段对应 `AL_iLQR_Tutorial.pdf` 和 `ALTRO` 论文中的 augmented Lagrangian 形式：

- 等式约束：
  `lambda^T c + 0.5 * mu * c^2`

- 不等式约束：
  使用投影到非负正交锥的形式处理

同时也对应：

- dual update
- penalty update

## 为什么这一阶段还不直接做完整 AL 外循环

因为完整 `AL-iLQR` 包含两个循环：

- 内层 `iLQR`
- 外层 `lambda / mu` 更新

如果现在一起上，很难区分：

- augmented cost 是否写对
- dual / penalty update 是否写对
- outer loop 终止条件是否写对

所以本阶段先停在“单个 AL 子问题可构造、可求解、可更新”。

## 验收标准

- AL cost 在约束违反时高于原始 cost
- dual update 后乘子发生变化
- penalty update 后最大罚参数增大
- 可以把 AL 子问题交给现有 `iLQR` 求解
