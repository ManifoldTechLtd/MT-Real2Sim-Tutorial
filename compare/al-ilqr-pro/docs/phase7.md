# Phase 7

## 目标

在 `Phase 6` 已经能构造固定 `lambda, mu` 的 augmented Lagrangian 子问题基础上，补齐完整的 `AL-iLQR` 外循环。

这一阶段的核心不是再发明新的内层求解器，而是把两个层次真正接起来：

- 外层：更新 `dual` 和 `penalty`
- 内层：调用现有稳定版 `iLQR` 求解当前 AL 子问题

## 本阶段实现的功能

- 增加 `ALILQRSolver`
- 增加 `ALILQROptions`
- 增加 outer iteration 日志结构
- 支持完整流程：
  - 用当前 `lambda, mu` 构造 AL 子问题
  - 调用内层 `iLQR`
  - 统计 base cost / augmented cost / max violation
  - 更新 dual variables
  - 在违反量下降不足时增长 penalty
- 提供 `phase7_al_ilqr_demo`
- 提供 `phase7_al_ilqr_test`

## 对应理论

这一阶段对应 `AL_iLQR_Tutorial.pdf` 中完整的 `AL-iLQR` 求解框架，也对应 `ALTRO` 论文里外层 augmented Lagrangian 循环的基本思路。

可以把这一阶段的代码直接映射到下面的理论步骤：

1. 固定 `lambda, mu`
   解一个无约束的 `iLQR` 子问题

2. 用新轨迹评估约束违反量

3. 更新乘子
   - 等式约束：`lambda <- lambda + mu * c`
   - 不等式约束：`lambda <- max(0, lambda + mu * c)`

4. 当违反量下降不足时增长罚参数
   - `mu <- phi * mu`

5. 若违反量低于容差，则停止外循环

## 当前实现的取舍

这一阶段仍然是教学版 `AL-iLQR`，不是完整的 `ALTRO` 复刻。

当前实现有几个明确取舍：

- 内层仍然是我们自己的有限差分 `iLQR`
- penalty update 使用简单的“下降不足则放大”规则
- 终止条件目前以最大约束违反量为主
- 还没有进入自动驾驶道路/障碍物建模

这些取舍是刻意的。目标是先把最核心的 `AL solve-update cycle` 讲清楚，并且让它稳定可运行。

## 验收标准

- `ALILQRSolver` 可以完成多轮 outer iteration
- 外层日志可以输出 violation、penalty 和 inner iteration 次数
- 在可达的约束问题上，最大违反量明显下降
- demo 可以输出一条最终轨迹
- 测试通过
