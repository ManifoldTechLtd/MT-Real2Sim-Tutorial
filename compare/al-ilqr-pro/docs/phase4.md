# Phase 4

## 目标

把 `Phase 3` 的基础版 `iLQR` 升级成更稳定的版本。

本阶段关注的不是新的最优控制结构，而是数值稳定性：

- line search
- expected decrease
- regularization

## 本阶段实现的功能

- 给 `Quu` 增加 `rho I` 正则化
- 当 backward pass 失败或 forward pass 不接受时增大 `rho`
- 当一步成功时减小 `rho`
- 实现 backtracking line search
- 记录每次迭代的 `alpha`、`rho`、improvement ratio
- 提供 `phase4_unicycle_ilqr_stable_demo`

## 对应理论

本阶段直接对应 `AL_iLQR_Tutorial.pdf` 中的两部分：

- forward pass line search
- backward pass regularization

也就是从“理论上能收敛的局部二次近似”走向“实际代码里更稳定的求解器”。

## 为什么这一阶段重要

`Phase 3` 的基础版 `iLQR` 已经能下降，但它更像“算法骨架”。

真实问题里常见两类失败：

- `Quu` 条件很差，甚至不可逆
- 全步长更新让 cost 上升

所以必须引入：

- `Quu + rho I`
- 逐步缩小 `alpha`
- 用实际下降 / 期望下降比值判断这一步是否可信

## 验收标准

- 迭代日志中能看到 `alpha` 和 `rho`
- cost 单调下降或总体下降
- 更难一点的初值下仍然能稳定收敛
- 测试通过
