# Phase 2

## 目标

从“给定控制序列做前向评估”进入“根据代价函数反向求最优反馈律”。

本阶段的关键词是：`Bellman 递推`、`Riccati 方程`、`有限时域离散 LQR`。

## 本阶段实现的功能

- 增加 `FiniteHorizonLQRSolver`
- 支持有限时域离散 LQR 的 backward recursion
- 计算每个时刻的时变反馈增益 `K_k`
- 生成闭环轨迹
- 导出轨迹到 CSV
- 提供一个最小 Python 绘图脚本用于可视化

## 对应理论

本阶段对应的是经典离散 LQR：

- 线性动力学 `x_{k+1} = A x_k + B u_k`
- 二次型代价
- cost-to-go 二次形式
- Riccati 逆向递推
- 闭环控制律 `u_k = u_ref + K_k (x_k - x_ref)`

这一阶段的意义是：

- 首次引入 backward recursion
- 为后续 `iLQR` 的 backward pass 建立最小、可验证的模板

## 为什么这一阶段就加入简单可视化

可以加，但不应该喧宾夺主。

本阶段的可视化只承担一个目的：帮助观察闭环轨迹是否向目标收敛。  
所以采用最轻量的方式：

- C++ example 输出 CSV
- Python 脚本读取 CSV 并画图

这样不会把图形依赖耦合进求解器核心。

## 验收标准

- LQR 求解器能输出非零反馈增益
- 闭环轨迹比初始状态更接近目标
- 测试通过
- 可以导出轨迹 CSV 并绘图
