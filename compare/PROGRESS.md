# 导航方案对比 — 工作进度

_最后更新: 2026-06-12_

## 项目目标

在 Unitree Go2 四足机器人上对比 **4 种导航后端**，量化各方案在安全性、速度、平滑性、动态避障 4 个维度的差异：

| 方案 | 简称 | 状态 |
|---|---|---|
| A — ETH-SRU (端到端 RL) | `eth-sru` | 分析完成，实机暂缓 |
| B — vanilla NeuPAN (hanruihua 原版) | `neupan-v` | **已部署，待实机验收** |
| C — manifold NeuPAN fork (当前线上) | `neupan-c` | 已部署运行中 |
| D — AL-iLQR (显式约束 iLQR) | `al-ilqr` | 算法分析完成，ROS 封装待实现 |

---

## 已完成工作

### 分析文档

| 文件 | 内容 |
|---|---|
| `compare/eth-sru-analysis.md` | 方案 A 包结构、接口、避障逻辑、与 OAYN 兼容性 |
| `compare/odin-nav-stack-analysis.md` | 方案 C 完整分析 (8 节)，含 fake360/Smith/stuck-escape 等 |
| `compare/al-ilqr-pro-analysis.md` | 方案 D 算法分析，AL 外层 + iLQR 内层 + 多圆碰撞 |

### 狗上实际部署

#### 方案 C: OAYN manifold NeuPAN fork (已可用)

```
/home/unitree/code/OAYN/
├── NeuPAN/                  # ManifoldTehLtd/NeuPAN fork
├── ros_ws/                  # catkin ws (map_planner, fake360, relocalization, unitree_control…)
└── scripts/                 # bag_record, pub_manual_waypoints…
```

启动: 见 `compare/测试手册.md` §方案C 节

---

#### 方案 B: hanruihua vanilla NeuPAN (新部署, **待实机验收**)

```
/home/unitree/code/NeuPAN-vanilla/
├── algo/                    # hanruihua/NeuPAN @579e7af (Python package only, no ros)
│   └── example/dune_train/model/go2/model_5000.pth   # Go2 DUNE checkpoint
├── ros_ws/src/neupan_ros/   # hanruihua/neupan_ros @b9558f3
├── configs/
│   └── neupan_planner_go2_vanilla.yaml   # vanilla defaults + Go2 robot 尺寸
├── launch/
│   └── neupan_vanilla_go2.launch         # 参考用 (env roslaunch 坏了, 目前用脚本)
├── run_neupan_vanilla.sh    # 入口 wrapper (直接 python 起节点, 绕过坏的 roslaunch)
└── README.md                # 完整踩坑记录
```

Dry-run 已通过 (见 README.md)。**待实机跑一次完整路径**。

---

### 部署物本地副本

`compare/方案B-NeuPAN-vanilla/` — configs / launch / run_neupan_vanilla.sh / README 的 dev 机副本。

---

## 进行中 / 下一步

### 优先任务 (短期)

1. **方案 B 实机验收** — 在走廊环境跑一遍 `run_neupan_vanilla.sh`，录 bag。
   - 验收标准: 节点启动无报错，robot 能跟 `/initial_path` 走，发出合理 `/cmd_vel`，无撞障。

2. **方案 B vs C 对比 bag 录制** — 按 `compare/测试手册.md` 完成 3 个场景录制。

### 中期任务

3. **方案 D: AL-iLQR ROS wrapper**
   - 写 `ilqr_local_planner` 节点 (Python/C++)
   - 订: `/initial_path`, `/scan_360`, `/odin1/odometry`
   - 发: `/cmd_vel`, `/neupan/arrive`
   - 发 AL-iLQR 需要外部动态预测模块 (CV/Kalman for dynamic obs)

4. **方案 A: ETH-SRU 评估**
   - 独立基线，不与 OAYN 前端兼容
   - 按 `eth-sru-analysis.md` §6 部署建议操作
   - 可在仿真环境 (IR-SIM) 做对比，不一定要上实机

### 延后评估

5. **地图规划 Hybrid A***  — map_planner 已做 heading-aware A*，增量价值低，暂缓
6. **GPU NeuPAN** — cvxpy CPU-only, 实测 runtime 不受益，暂缓

---

## 关键技术细节 (备忘)

### neupan mamba env 状态 (重要)

- `pip show neupan` → editable 指向 `/home/unitree/code/NeuPAN` (EnderMandS fork)
- env 的 `rospack`/`roslaunch` **坏了** (libboost_filesystem.so.1.86 未装入 env/lib)
- 运行 vanilla 通过 `PYTHONPATH` 前置覆盖, 直接 `python neupan_node.py` 起节点
- **不要 `mamba install` 任何东西到 neupan env** — 会风险污染现有 fork

### 方案 B 与方案 C 的算法差异

vanilla 没有这些 manifold 增量:

| 增量 | 说明 |
|---|---|
| `q_theta` / `p_w` / `p_dw` | manifold 加的代价权重 |
| B11 set_last_omega | cross-cycle ω anchor |
| F2 Smith predictor | 控制延迟补偿 |
| F1 odom_feedback | MPC 用实测速度而非 cmd |
| B5 reset_velocity=False | 重置时保留 cur_vel (防 stop-go) |
| stuck-escape recovery | 卡住后退+旋转 |
| /neupan/arrive auto-replan | 到达后自动 replan |

### 两个方案都用的前端 (OAYN ros_ws)

- `map_planner` (heading-aware A*) → `/initial_path`
- `fake360` → `/scan_360` (360° 假 LaserScan, 障碍物持久化)
- `relocalization` (可选)
- `unitree_control` → `/cmd_vel` → Go2

---

## 文件索引

```
compare/
├── PROGRESS.md                    ← 本文件
├── 测试手册.md                    ← 傻瓜式启动+录 bag+结果提取指南
├── eth-sru-analysis.md            ← 方案 A 分析
├── odin-nav-stack-analysis.md     ← 方案 C 分析
├── al-ilqr-pro-analysis.md        ← 方案 D 分析
├── 执行计划.md                    ← 原始任务列表 (历史)
├── 方案B-NeuPAN-vanilla/          ← 方案 B 部署物本地副本
│   ├── README.md
│   ├── configs/
│   ├── launch/
│   └── run_neupan_vanilla.sh
└── scripts/
    └── record_bag.sh              ← 统一 bag 录制脚本
```
