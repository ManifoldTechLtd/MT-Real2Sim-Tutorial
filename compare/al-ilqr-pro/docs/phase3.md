# Phase 3

## 目标

实现第一个可运行的无约束 `iLQR`。

这一阶段的重点不是把所有工程细节都做满，而是把 `iLQR` 的核心闭环先跑通：

- rollout
- local expansion
- backward pass
- forward pass

## 本阶段实现的功能

- 增加 `ILQRSolver`
- 使用有限差分计算 dynamics Jacobian 和 cost 导数
- 实现无约束 `iLQR` 的基础 backward pass
- 实现基础 forward pass
- 提供 `phase3_unicycle_ilqr_demo`
- 导出优化后轨迹 CSV
- 提供最小绘图脚本

## 对应理论

本阶段直接对应 `AL_iLQR_Tutorial.pdf` 中无约束 `iLQR / DDP` 的主体：

- 动力学一阶展开
- cost 二阶展开
- action-value 二次近似
- 反馈项 `K_k`
- 前馈项 `d_k`
- 用更新后的控制重新 rollout

## 当前实现的边界

为了让教学路径清晰，本阶段采用的是“基础可运行版”：

- dynamics 和 cost 导数通过有限差分获得
- forward pass 使用一组固定 `alpha` 候选来选更优轨迹

这意味着：

- 它已经具备 `iLQR` 的核心结构
- 但还没有进入更完整的 line search 和 regularization

这些增强将在 `Phase 4` 完成。

## 为什么这里先不用解析导数

因为当前目标是先把算法结构讲清楚，而不是先做工程优化。

有限差分的好处是：

- 不要求每个模型都先写 Jacobian
- 更适合教学阶段快速验证公式和数据流

代价是：

- 速度更慢
- 数值精度依赖扰动步长

## 验收标准

- `iLQR` 可以让总代价下降
- 优化后的轨迹比初始轨迹更接近目标
- example 可以输出 cost history
- 可以导出轨迹并画图
