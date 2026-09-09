# Phase 1

## 目标

把 `Phase 0` 的最小骨架扩展成真正可复用的基础层，使它能承载后续 `iLQR` 的实现。

## 本阶段实现的功能

- 把代价抽象成 `CostFunction` 接口
- 保留 `QuadraticCost` 作为第一个具体实现
- 增加 `UnicycleModel` 非线性离散动力学
- 增加 `KinematicBicycleModel` 运动学自行车模型
- 继续复用 `OptimalControlProblem` 做 rollout 和总 cost 评估
- 增加 `phase1_unicycle_rollout` example
- 增加 `phase1_bicycle_rollout` example
- 增加 `phase1_unicycle_test`
- 增加 `phase1_bicycle_test`

## 对应理论

本阶段仍然没有进入 backward pass / forward pass。

它主要对应离散最优控制问题里最基础、但后续每一步都会依赖的三件事：

- 非线性离散动力学 `x_{k+1} = f(x_k, u_k)`
- 阶段代价 `l(x_k, u_k)` 与终端代价 `l_f(x_N)`
- 在给定控制序列时，对轨迹和总 cost 的前向评估

这也是后面 `iLQR` 每次迭代都要反复做的事情。

其中自行车模型采用自动驾驶里常用的运动学单轨表达：

- 状态 `[x, y, yaw, v]`
- 控制 `[a, delta]`
- 运动学关系 `yaw_dot = v * tan(delta) / L`

## 为什么这一阶段要做接口抽象

后续 `AL-iLQR` 不会只用普通二次代价。

我们还会引入：

- augmented Lagrangian cost
- 约束罚项
- 终端约束相关代价

所以现在就要把 `OptimalControlProblem` 从“绑定 `QuadraticCost`”改成“绑定任意 `CostFunction`”。

## 验收标准

- 线性系统 example 仍然可运行
- 新增独轮车非线性 rollout example 可运行
- 新增自行车模型 rollout example 可运行
- 非线性模型测试通过
