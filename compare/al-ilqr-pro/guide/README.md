# AL-iLQR 轨迹优化开发指南

本指南面向零基础入门学生，从最基本的概念开始，一步步讲解如何从零构建一个完整的 **增广拉格朗日-迭代线性二次调节器（AL-iLQR）** 轨迹优化求解器，并将其应用于自动驾驶轨迹规划。

## 目录

| 章节 | 内容 | 对应代码阶段 |
|------|------|-------------|
| [Chapter 1](chapter1_basics.md) | 基础概念与项目骨架 | Phase 0-1 |
| [Chapter 2](chapter2_lqr.md) | 有限时域 LQR | Phase 2 |
| [Chapter 3](chapter3_ilqr.md) | 无约束 iLQR 求解器 | Phase 3-4 |
| [Chapter 4](chapter4_constraints.md) | 约束建模 | Phase 5 |
| [Chapter 5](chapter5_augmented_lagrangian.md) | 增广拉格朗日方法 | Phase 6 |
| [Chapter 6](chapter6_al_ilqr.md) | 完整 AL-iLQR 求解器 | Phase 7 |
| [Chapter 7](chapter7_autodrive.md) | 自动驾驶场景应用与碰撞建模 | Phase 8-9 |
| [Chapter 8](chapter8_extensions.md) | 工程增强与扩展 | Phase 10-13 |
| [Chapter 9](chapter9_polygon_obstacles.md) | 多边形障碍物碰撞约束 | Phase 14 |
| [Chapter 10](chapter10_warmstart_and_initial_guess.md) | 初始解策略：PP Warmstart 与 Guidance Tracking Cost 分析 | PP / Tracker |
| [最终篇](chapter_final_overview.md) | 项目全览：构建、运行与完整调用链 | 全部 |
| [附录：论文步骤与代码对照](appendix_paper_alignment.md) | 按论文步骤逐项对照源码实现 | 参考附录 |

## 阅读建议

- 按顺序阅读，每章依赖前一章的知识
- 每章开头有"基础概念"部分，不熟悉可以先读
- 代码引用均来自本项目 `src/` 和 `include/` 目录
- 参考论文：`paper/AL_iLQR_Tutorial.pdf` 和 `paper/ALTRO A Fast Solver for Constrained Trajectory Optimization.pdf`

> **注意**：本仓库实现的是 **AL-iLQR 主线算法**，不是完整的 ALTRO 求解器。本项目与论文存在多处实现差异（数值差分代替解析 Jacobian、条件性惩罚更新 heuristic、iLQR 而非 DDP、标准 AL 形式而非 conic 形式等），完整差异清单参见 [Chapter 8 §与论文的差异汇总](chapter8_extensions.md#与论文的差异汇总)。

## 项目全局依赖关系

```
Chapter 1: Trajectory + DynamicsModel + CostFunction + OCP      ← 能仿真、能评估
    │
Chapter 2: FiniteHorizonLQRSolver                               ← 能求解线性问题
    │
Chapter 3: ILQRSolver (展开 + backward + forward + 正则化)       ← 能求解非线性无约束问题
    │
Chapter 4: ConstraintFunction + ConstrainedOCP                  ← 能建模约束
    │
Chapter 5: AugmentedLagrangianCost                              ← 能将约束转为代价
    │
Chapter 6: ALILQRSolver                                         ← 能求解带约束问题 (核心完成!)
    │
Chapter 7: 自动驾驶场景 (碰撞建模 + 场景组装)                      ← 能应用于实际问题
    │
Chapter 8: 数值增强 + 动态障碍物 + 车体几何                       ← 更鲁棒、更真实
    │
Chapter 9: 多边形障碍物 (LSE 光滑距离 + 精确碰撞检测)              ← 支持任意凸多边形障碍物
```
