# Development Guide Outline

本文件定义本项目的正式开发指南大纲。

目标不是把 `AL-iLQR` 写成一份纯理论教材，而是把“理论、代码、运行现象、阶段验收”按工程开发顺序串起来，指导读者从零完成一个可运行、可验证、可视化的 `AL-iLQR` 轨迹优化项目。

## 写作原则

每一章都必须满足以下要求：

1. 对应一个独立的小型 CMake 工程
2. 对应一份独立的章节文档
3. 当前章节只引入当前章节理解所必需的最小代码
4. 每章都提供明确的运行命令和验收标准
5. 理论解释必须和代码文件、运行结果、调试现象对齐

## 统一章节模板

每一章按照统一模板组织：

1. 本章目标
2. 本章完成后会得到什么
3. 对应小工程目录
4. 理论最小集
5. 代码实现
6. 关键文件解读
7. 运行命令
8. 验收标准
9. 常见问题
10. 本章小结

## 教程章节与工程映射

### Chapter 0: 工程骨架

对应工程：
- `tutorial/chapter_00_project_skeleton`

目标：
- 搭出最小 CMake 工程
- 明确目录、example、test 的组织方式

理论最小集：
- 本章不引入核心算法理论
- 只讲工程组织原则

验收：
- 能单独配置、编译、运行

### Chapter 1: 问题表达与 Rollout

对应工程：
- `tutorial/chapter_01_problem_and_rollout`

目标：
- 把离散时间最优控制问题表达成代码对象
- 完成状态、控制、轨迹、动力学、代价、rollout 的最小闭环

理论最小集：
- 离散时间最优控制建模
- 状态转移 `x_{k+1} = f(x_k, u_k)`
- 阶段代价与终端代价
- rollout 与总代价

验收：
- 给定控制序列能 rollout
- 能正确计算总代价

### Chapter 2: 动力学模型

对应工程：
- `tutorial/chapter_02_vehicle_models`

目标：
- 从简单系统走到自动驾驶常用自行车模型

理论最小集：
- point mass
- unicycle
- kinematic bicycle
- 后轴中心作为规划点

验收：
- 各模型都能单独 rollout
- 自行车模型行为正确

### Chapter 3: 有限时域 LQR

对应工程：
- `tutorial/chapter_03_finite_horizon_lqr`

目标：
- 建立线性最优控制基线

理论最小集：
- Bellman 原理
- Riccati 递推
- 时变反馈增益

验收：
- 线性系统闭环稳定
- 与理论趋势一致

### Chapter 4: 无约束 iLQR

对应工程：
- `tutorial/chapter_04_unconstrained_ilqr`

目标：
- 实现最小可运行 `iLQR`

理论最小集：
- dynamics 一阶展开
- cost 二阶展开
- backward pass
- forward pass
- `k` 和 `K`

验收：
- cost 持续下降
- 非线性轨迹明显改进

### Chapter 5: iLQR 稳定化

对应工程：
- `tutorial/chapter_05_ilqr_stabilization`

目标：
- 从“能跑”升级到“稳定能跑”

理论最小集：
- line search
- regularization
- expected decrease
- 失败判据

验收：
- 较差初值下仍然能稳定运行

### Chapter 6: 约束建模

对应工程：
- `tutorial/chapter_06_constraints`

目标：
- 独立讲清约束表达与 violation 评估

理论最小集：
- 等式约束
- 不等式约束
- stage / terminal constraint
- violation 的定义

验收：
- 能单独评估轨迹约束
- 能统计最大违反量

### Chapter 7: 增广拉格朗日 Cost

对应工程：
- `tutorial/chapter_07_augmented_lagrangian_cost`

目标：
- 讲清 AL 如何把约束并入代价

理论最小集：
- augmented Lagrangian
- dual variable
- penalty
- equality / inequality update

验收：
- 固定乘子和罚参数时，子问题可正常求解

### Chapter 8: 完整 AL-iLQR

对应工程：
- `tutorial/chapter_08_al_ilqr_solver`

目标：
- 形成完整 `AL-iLQR` 外循环

理论最小集：
- outer loop
- dual update
- penalty schedule
- best-so-far trajectory

验收：
- violation 逐轮下降
- 返回结果可解释

### Chapter 9: 自动驾驶场景

对应工程：
- `tutorial/chapter_09_autodrive_scenario`

目标：
- 把求解器放入自动驾驶场景

理论最小集：
- 参考线
- 道路边界
- 静态 / 动态障碍物
- 场景如何进入优化问题

验收：
- 能跑静态障碍物
- 能跑动态障碍物

### Chapter 10: 车辆几何与碰撞

对应工程：
- `tutorial/chapter_10_vehicle_geometry_and_collision`

目标：
- 从点车模型升级到多圆车体模型

理论最小集：
- 后轴中心规划点
- 车体矩形
- 多圆拟合
- 多圆 vs 圆障碍物碰撞

验收：
- 改圆数时碰撞几何发生变化
- 配置文件生效

### Chapter 11: 交互式前端与调试可视化

对应工程：
- `tutorial/chapter_11_interactive_visualization`

目标：
- 做一个真正服务教学与调试的单窗口前端

理论最小集：
- 这章以工程组织和调试方法为主
- 重点讲清“为什么显示这些信息”

验收：
- 调参数立即更新预览
- 点击规划显示结果与日志

### Chapter 12: 调试方法与常见问题

对应工程：
- `tutorial/chapter_12_debugging_and_validation`

目标：
- 教会读者定位问题，而不是只会运行

理论最小集：
- 如何看 `base_cost_history`
- 如何看 `violation_history`
- 如何看 `penalty_history`
- 如何区分代价、约束、数值、几何问题

验收：
- 能根据日志和轨迹现象定位问题方向

### Chapter 13: 从教学版到工程增强

对应工程：
- `tutorial/chapter_13_next_steps`

目标：
- 说明当前实现的边界和后续扩展路线

理论最小集：
- warm start
- 解析 Jacobian
- infeasible start
- slack variable
- MPC 化
- 向 ALTRO 靠近的方向

验收：
- 读者能清楚知道如何继续扩展项目

## 推荐写作顺序

为了最快形成一套可用教程，建议按以下顺序落笔：

1. Chapter 1: 问题表达与 Rollout
2. Chapter 2: 动力学模型
3. Chapter 4: 无约束 iLQR
4. Chapter 6: 约束建模
5. Chapter 7: 增广拉格朗日 Cost
6. Chapter 8: 完整 AL-iLQR
7. Chapter 9: 自动驾驶场景
8. Chapter 10: 车辆几何与碰撞
9. Chapter 11: 交互式前端与调试可视化
10. 再补其余章节

## 当前启动项

当前正式开始的章节：
- Chapter 1: 问题表达与 Rollout

下一步工作：
1. 搭建 `tutorial/chapter_01_problem_and_rollout` 的独立工程
2. 写 Chapter 1 的正文草稿
3. 明确本章的最小代码、最小测试和最小验收输出
