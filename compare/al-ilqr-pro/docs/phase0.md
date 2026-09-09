# Phase 0

## 目标

建立一个独立于参考仓库的最小教学骨架，先把“问题在代码里如何表达”这件事做对。

## 本阶段实现的功能

- `Trajectory`：状态和控制轨迹容器
- `DynamicsModel`：离散动力学接口
- `LinearPointMassModel`：最小线性系统示例
- `QuadraticCost`：阶段代价和终端代价
- `OptimalControlProblem`：rollout 和 total cost 计算
- `phase0_linear_rollout`：可运行 example
- `phase0_smoke_test`：最小验证测试

## 对应理论

这一阶段还没有进入 `iLQR` 或 `AL-iLQR` 的核心递推。

主要对应的是离散时间最优控制问题的基本组成：

- 状态 `x_k`
- 控制 `u_k`
- 离散动力学 `x_{k+1} = f(x_k, u_k)`
- 阶段代价 `l(x_k, u_k)`
- 终端代价 `l_f(x_N)`

## 验收标准

- 可以编译整个工程
- example 可以输出一条状态轨迹和总 cost
- smoke test 可以验证 rollout 的终端状态是否正确
