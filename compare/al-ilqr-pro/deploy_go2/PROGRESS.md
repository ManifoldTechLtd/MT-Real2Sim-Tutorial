# AL-iLQR Go2 项目开发复盘日志（PROGRESS.md）

> 记录 al-ilqr-pro 适配宇树 Go2 机器人全过程：开发、修改、部署与测试复盘。
> 时间范围：2026-07-25 ~ 2026-07-30
> 部署包位置：`compare/al-ilqr-pro/deploy_go2/`（自包含 catkin 包，包名 `al_ilqr_go2`）

---

## 2026-07-25：初始适配与部署包创建

### 1. 代码调研

- **工作内容**：分析 al-ilqr-pro 原始仓库结构与可复用组件。
- **如何发现/触发原因**：项目启动，需评估适配 Go2 的可行性与工作量。
- **调研要点**：
  - 原仓库核心：AL-iLQR 求解器、`ros_wrapper/`（含 `guidance_tracker`）、动力学模型 `KinematicBicycleModel`；
  - `bicycleToDiff` 映射：自行车模型输出（v, δ）→ 差速/四足底盘（v, ω）的转换逻辑可直接复用；
  - Odin-Nav-Stack 中 `map_planner`（全局规划）、`fake360`（点云聚合）、`unitree_control`（底盘控制）可作为上下游复用组件。
- **验证结果**：确认技术路线可行，输出适配方案。
- **备注/遗留**：无。

### 2. 部署包创建（Task 1）

- **工作内容**：创建 `deploy_go2/` 自包含 catkin 包。
- **触发原因**：部署包需自包含（源码复制而非引用上级目录），便于整体 rsync 到 NX。
- **做的修改**：
  - 从 al-ilqr-pro 复制 40+ 核心源文件（求解器、模型、约束、cost、guidance_tracker 等）到 `deploy_go2/`；
  - 编写 `CMakeLists.txt`（包名 `al_ilqr_go2`）与 `package.xml`。
- **验证结果**：包结构完整，本地可通过 catkin 构建。
- **备注/遗留**：无。

### 3. 动态避障接入（Task 2）

- **工作内容**：为跟踪器接入激光动态障碍物约束。
- **触发原因**：原仓库仅有静态场景示例，真机需实时避障。
- **做的修改**：
  - `guidance_tracker.hpp/cpp`：`Track()` 接口新增 `DynamicObstacle` 参数；
  - `al_ilqr_ros_node.cpp`：新增 `/scan_360` 订阅、欧式聚类提取障碍物、注入 `MultiCircleVehicleObstacleConstraint`。
- **验证结果**：编译通过，约束注入链路打通。
- **备注/遗留**：订阅话题后续改为 `/scan`（见 07-27）。

### 4. 配置与 Launch（Task 3）

- **做的修改**：
  - 新建 `config/al_ilqr_go2.yaml`：Go2 保守参数（限速、限角速度、避障半径等）；
  - 新建 `launch/al_ilqr_go2.launch`。
- **验证结果**：launch 可正常加载参数。

### 5. 文档与脚本（Task 4）

- **做的修改**：
  - `DEPLOY.md`：部署说明初版；
  - `install_deps.sh`：依赖安装脚本；
  - `calibrate_wheelbase.py`：等效轴距标定脚本。
- **验证结果**：文档与脚本齐备。

### 6. 代码审查修复

- **问题现象**：审查发现 launch 参数命名空间错误、参数键名不一致。
- **如何发现**：交付前代码审查。
- **做的修改**：
  - `al_ilqr_go2.launch`：`rosparam` 移入 `<node>` 内，修正命名空间；
  - `dynamic_obstacles` 参数键名与代码读取处对齐；
  - 话题配置扁平化。
- **验证结果**：参数可被节点正确读取。

---

## 2026-07-27：链路修正与功能完善

### 7. /scan_360 → /scan 切换（Task 5-6）

- **问题现象**：`/scan_360` 在实际链路中不存在。
- **如何发现**：调研实际传感链路为 `cloud_slam → pointcloud_to_laserscan → /scan`。
- **做的修改**：
  - `al_ilqr_ros_node.cpp`：动态避障订阅改为 `/scan`，加入防重复订阅设计；
  - `whole.launch`：修正 pointcloud_to_laserscan 高度过滤（`min_height=-0.2`, `max_height=0.4`）。
- **验证结果**：话题链路匹配实际系统。
- **备注/遗留**：需真机验证 /scan 数据质量。

### 8. 道路边界约束禁用（Task 7-8）

- **问题现象**：`MultiCircleRoadBoundaryConstraint` 在室内场景可能产生错误约束。
- **如何发现**：调研确认该约束是绝对坐标系下的 y 向走廊约束，并非真实车道线检测，室内不适用。
- **做的修改**：
  - 新增 `enable_road_boundary` 参数开关，默认 `false`；
  - fake360 点云衰减时间调整为 6.0s。
- **验证结果**：默认关闭后不再引入无效约束。

### 9. 到达事件与链路打通（Task 9-10）

- **问题现象**：给点导航端到端链路存在缺口——无到达事件、终点不减速、超范围目标点无提示。
- **如何发现**：端到端链路核查。
- **做的修改**：
  - 实现 `/neupan/arrive` 发布（`std_msgs/Empty`）+ 到达判定逻辑；
  - `path_converter`：终点减速（末端 `ref_speed=0`）；
  - `map_planner`：目标点超出地图范围时报错增强。
- **验证结果**：编译通过，链路逻辑完整。

### 10. DEPLOY.md 重构为规范化测试手册（Task 11）

- **做的修改**：`DEPLOY.md` 重构为 8 Phase 结构：
  1. 安全规程 → 2. 环境检查 → 3. 编译部署 → 4. 静态验证 → 5. 车体运动测试 → 6. 集成导航测试 → 7. 参数调试 → 8. 故障排查。
- **验证结果**：确立为唯一规范化部署与测试手册，后续操作按此分步执行并记录。

---

## 2026-07-28：首次真机部署（NX）

### 11. NX SSH 部署 Phase 0-3（Task 12）

- **工作内容**：按 DEPLOY.md 在 Jetson NX 上执行 Phase 0-3。
- **执行结果**：
  - **Phase 1 环境检查**：全通过（Ubuntu 20.04.5、ROS Noetic、Eigen3/PCL/tf2 齐全）；
  - **Phase 2 编译**：`catkin build al_ilqr_go2` 成功；修正 overlay 配置（`catkin config --extend`）；
  - **Phase 3 静态验证**：发现 3 个问题：

| # | 问题现象 | 原因 | 处理 |
|---|---------|------|------|
| a | map_planner 报 "Start or goal is occupied" | 机器人定位落在地图占据区 | 需物理重定位（遗留） |
| b | 求解耗时 2188ms | 编译未开优化（-O0） | 改 Release 后降至 124ms，仍超 100ms 目标（遗留） |
| c | `/initial_path` 坐标系不匹配 | 路径为 map 系，跟踪器工作在 odom 系 | 待修复（遗留） |

- **备注/遗留**：(b)(c) 于 07-30 根治；(a) 待现场物理重定位。

---

## 2026-07-30：性能与正确性大修

### 12. 求解耗时根因诊断（Task 13）

- **问题现象**：Release 下仍 124ms/次，远超解析求导应有的量级。
- **如何发现**：逐层排查求导路径，确认解析求导从未生效，实际一直在跑数值差分。三层缺陷：
  1. yaml 中解析求导开关只是占位，代码从未读取；
  2. `GuidanceTracker` 未设置 `ALILQROptions` 对应字段；
  3. `TrackingStageCost` 缺少 `HasAnalyticalDerivatives()` 覆写，基类默认走数值差分。
- **做的修改**：补齐 `TrackingStageCost` 全部闭式导数（Jacobian/Hessian）、接通参数读取与 options 传递。
- **验证结果**：x86 实测 0.64ms/标称场景，预计 NX 上 1-5ms（待真机确认）。
- **备注/遗留**：NX 实际耗时待 Phase 3 复测。

### 13. 坐标系修复（Task 13）

- **问题现象**：`/initial_path`（map 系）直接被当作 odom 系使用，跟踪目标整体偏移。
- **如何发现**：07-28 Phase 3 静态验证发现，本次代码走查确认 `pathCallback()` 从未做 map→odom 变换。
- **做的修改**：`al_ilqr_ros_node.cpp` 新增 `transformPathToOdom()`，使用 TF2 实时查询 map→odom 并逐点变换。
- **验证结果**：编译通过；变换逻辑静态检查正确。
- **备注/遗留**：需真机 TF 树在线验证。

### 14. 全面代码检查（Task 13）

- **工作内容**：对整包做防御性走查，修复 10+ 处问题。
- **修复要点**：
  - 参数占位全部接通（之前多个 yaml 参数未被读取）；
  - 回调与主循环共享数据加锁（线程安全）；
  - 数值安全：除零保护、角度归一化；
  - 数据陈旧兜底：odom/path/scan 超时检测与降级停车；
  - NaN 防护：求解输出校验。
- **验证结果**：编译通过，0 warning。

### 15. 包移植到 code_ilqr（Task 14）

- **工作内容**：将依赖包从 k02_nav 迁入独立工作区 `code_ilqr`，实现依赖隔离。
- **做的修改**：
  - `map_planner`、`fake360`、`unitree_control` 复制到 `code_ilqr/src/`（`odin_ros_driver` 保持在 k02 工作区）；
  - `whole.launch` 参数按 DEPLOY.md 手册调整。
- **验证结果**：4 包编译通过，0 warning。

### 16. 修复代码同步到 NX（Task 15）

- **做的修改**：rsync 11 个变更文件到 NX `~/code_ilqr`。
- **验证结果**：`catkin build`（Release）成功，耗时 21.8s。

---

## 当前状态（截至 2026-07-30）

### 已就绪

- NX 工作区 `~/code_ilqr`：`al_ilqr_go2` + `map_planner` + `fake360` + `unitree_control`；
- 全部编译通过（Release -O3，0 warning）；
- 解析求导已接通（x86 实测 0.64ms）；
- map→odom TF2 变换已实现；
- 到达事件、终点减速、动态避障（/scan）、道路边界开关等功能链路完整。

### 阻塞项

- 机器人定位落在地图占据区，需**物理重定位**后才能验证真实路径闭环（对应 07-28 问题 a）。

### 待验证

| 项目 | 说明 |
|------|------|
| 解析求导 NX 实际耗时 | 预计 1-5ms，需 Phase 3 复测确认 |
| map→odom 变换正确性 | 需真机 TF 树在线验证路径对齐 |
| 给点导航端到端闭环 | Phase 5/6 车体运动与集成导航测试尚未执行 |

### 下一步计划

1. 现场物理重定位机器人至地图空闲区；
2. 复跑 DEPLOY.md Phase 3 静态验证（求解耗时、路径坐标）；
3. 执行 Phase 5 车体运动测试 → Phase 6 集成导航测试；
4. 按 Phase 7 调参并记录结果。

---

### 2026-07-30 续：Phase 3 静态闭环 + Phase 4 初步车体测试

**17. 静态闭环验证成功**
- 问题现象：上一轮 map_planner "Start or goal is occupied" 
- 发现原因：机器人位置在地图膨胀区（需物理重定位到空旷区）
- 操作：用户遥控 Go2 到走廊中间，重新发目标点 (-20, 3.8)
- 结果：/initial_path 成功生成；al_ilqr status 全部 OK cvg=1，求解 2.1-2.5ms，解析导数确认生效，obs=5-9
- 验证通过：坐标系变换正确（map→odom TF 实时生效），求解耗时从 124ms 降到 2ms

**18. 间歇性求解不收敛问题**
- 问题现象：al_ilqr 日志大量 "solver did not converge"（约每 1-2s 一次），导致 cmd_vel 在小正值和零之间交替，机器人上半身前后抽搐
- 发现原因：/scan 检测到 8-10 个走廊墙壁障碍物，每个在 20 stage 各加约束（200 条），对 max_outer=4/max_inner=12 约束太紧
- 修改：NX yaml 改为 max_outer=6, max_inner=30, max_obstacles=5
- 结果：收敛率 5/5（100%），不再抽搐
- 备注：本机仓库 yaml 的默认值未改（仍是 4/12/10），后续需评估是否同步修改默认值

**19. 前进速度接近零问题（当前阻塞项）**
- 问题现象：求解器收敛但 a≈-0.001（几乎零加速度），cmd_vel linear.x ≈ 0.001 m/s（Go2 死区内），机器人不走
- 伴随现象：steering=0.142（想转向但不前进）；说明求解器有转向动力但无前进动力
- 疑似根因：guidance 中 ref_speed 未正确传入 TrackingStageCost 代价函数，导致求解器没有"要以 ref_speed 前进"的驱动力
- 未修复，待下一轮代码检查

**20. Phase 4 手动 cmd_vel 底盘验证**
- 操作：杀掉 al_ilqr，直接 rostopic pub /cmd_vel linear.x=0.2 持续 2s
- 结果：Go2 确实向前冲了一段（底盘链路完整验证通过）
- 结论：问题在规划器输出太小，不在底盘执行链路

### 当前状态
- NX 所有程序已关闭，Go2 已趴下
- 阻塞问题：求解器输出零加速度（ref_speed 未驱动代价函数）
- 下一步：修复 path_converter 的 ref_speed 赋值 / TrackingStageCost 的速度跟踪代价

---

## 2026-07-30 续2：前进速度近零问题根因定位与修复（本机，待真机验证）

**21. 根因定位：推翻“ref_speed 未传入代价函数”假设**
- 如何发现：逐层走查 ref_speed 链路——yaml `planning/ref_speed=0.8` → `loadParameters()` → `PathConverter::configure()` → `extractGuidance()` 中 `gp.speed = ref_speed_/(1+k·κ)` 赋值正确 → `TrackingStageCost::StageCost()` 中确有 `0.5·spd_w·(v - ref.speed)²` 项且解析导数一致。**原疑似根因不成立**
- 真正根因（三问题叠加）：
  1. **墙壁外接圆堵死走廊（主因）**：`ObstacleAdapter` 把整段走廊墙聚成一个簇后取外接圆，圆心在墙面上、半径可达 1.7m（+0.3m 膨胀），圆体侵入走廊中央——任何前进都违约，“原地不动”成为零违约最优解。与真机现象完全吻合：cvg=1、viol=0.000、steering=0.142（想绕）但 a≈-0.001（不想前进）
  2. **warm start 自锁**：一旦输出近零加速度解，下帧 warm start 继续以该解起步（steering 项使 norm²>1e-8 恒成立，Pure-Pursuit 前向初值永不再生效），求解器锁死在静止局部最优
  3. **速度自举死锁**：`bicycleToDiff` 的 `v_cmd = odom速度 + a·dt`，静止时 odom 速度=0，即使 a=1.0 也只有 0.1 m/s（Go2 死区内）→ 机器人不动 → odom 恒 0 → 永远爬不出死区

**22. 修复内容（均已本机 Docker noetic 验证编译通过，0 warning）**
- `obstacle_adapter.hpp`：新增大簇拆分——空间跨度超过 `2×max_cluster_radius`（默认 0.4m）的簇拆为多个小子簇，墙壁变为贴墙小圆串，走廊中央保持可通行；新参数 `dynamic_obstacles/max_cluster_radius`
- `bicycle_to_diff.hpp` + `al_ilqr_ros_node.cpp`：cmd_vel 线速度改为取规划轨迹前视第 k 步速度 `states[k](3)`（新函数 `bicycleToDiffFromSpeed`，新参数 `control_mapping/speed_lookahead_steps=3`，起步指令直接 ≈ 0.3 m/s 脱离死区；取 0 退化回旧行为）
- `al_ilqr_ros_node.cpp`：新增停滞恢复——求解收敛但 |cmd_v|<0.05 且未接近路径末端，连续 3 周期后清空 warm start，下周期回退 Pure-Pursuit 初值（含前向加速度）跳出静止局部最优；新参数 `planning/stall_cmd_threshold`、`planning/stall_recovery_cycles`
- `al_ilqr_go2.yaml`：同步 07-30 NX 实测值为默认值（max_outer=6、max_inner=30、max_obstacles=5），新增上述 3 个新参数及注释
- 验证结果：`algo-warehouse-ros1-dev` 镜像内 catkin_make Release 全量编译通过，0 warning
- 备注/遗留：待 Go2 开机后 rsync 到 NX 重新编译，按 DEPLOY.md Phase 3→4→5 验证：预期 obs 数量变多但单个变小（贴墙小圆），cmd_vel 起步 ≥ 0.2 m/s

### 当前状态（截至 2026-07-30 续2）
- 三项修复已完成并本机编译验证，**尚未同步到 NX**（Go2 关机中）
- 下一步：Go2 开机后 → rsync → NX 编译 → 重定位 → 静态验证 cmd_vel 输出 → Phase 4/5 真机测试

---

## 2026-07-30 续3：真机验证——端到端导航一次性闭环成功

**23. 部署与静态验证**
- 操作：rsync 修复代码到 NX → catkin build Release（16.8s，0 warning）→ 启动 odin driver（重定位成功，map→odom TF 非平凡变换确认）→ 用户遥控机器人到走廊中央空旷区（离墙约 1m）→ 启动 whole.launch + al_ilqr_go2
- 新参数全部生效确认：max_cluster_radius=0.4、speed_lookahead_steps=3、stall_cmd_threshold=0.05、solver 6/30、max_obstacles=5
- 发目标点 (-20, 3.8) 后 /al_ilqr/status：
  - **a=1.005（满加速度前进）、v=0.53-0.62 m/s（远超死区）**，对比修复前 a≈-0.001、v≈0.001
  - cvg=1，iter 仅 (1/3)，求解 0.7-0.9ms；obs=5 贴墙小圆，min_r≈1.5-1.8m（与实际离墙距离一致），viol≤0.005
  - 三项修复（墙壁拆分/前视速度/停滞恢复）全部验证有效

**24. 意外的端到端真机闭环（发现重要安全隐患 + 集成测试一次通过）**
- 问题现象：预期的“静态验证”（use_unitree_control:=false，机器人不应该动）中，Go2 真实向前走了 5 米多
- 根因：**host_sdk_sample 节点随 odin_ros_driver 的 odin1_ros1.launch 自动启动，直接订阅 /cmd_vel 转发 Go2 SDK**——use_unitree_control:=false 只挡 code_ilqr 的 unitree_control，挡不住这条链路，底盘实际常通
- 安全处置：rosnode kill al_ilqr_go2_node + 零速度覆盖，机器人停止
- **但导航结果完全正确**：机器人从 (-19.7, -2.0) 自主导航到 (-19.75, 3.15)，行驶约 5.2m，日志确认 `goal reached (dist=0.293 m < 0.300 m), published arrive on /neupan/arrive`，到达后自动零速停车
- 结论：**Phase 5/6 集成导航测试实质性一次通过**——全局规划→TF变换→iLQR跟踪→避障→起步脱死区→到达停车→arrive 事件，全链路闭环
- 安全教训（已入经验库）：以后做静态验证必须先 `rostopic info /cmd_vel` 确认无订阅者，或把 al_ilqr 的 cmd_vel_topic 重映射到 /cmd_vel_test

### 当前状态（截至 2026-07-30 续3）
- 前进速度近零阻塞问题彻底解决，端到端给点导航真机验证通过
- NX 上所有 ROS 进程已全部关闭（含 odin driver、whole.launch、host_sdk_sample、rosout，验证残留进程数=0），等待下次测试指令
- 待办：多目标点/长距离/动态障碍测试（Phase 6 完整项）、Phase 7 参数精调、静态验证流程增加 cmd_vel 订阅者检查项

---

## 2026-07-31：测试辅助设施完善

**25. 傻瓜式测试手册 + bag 录制功能**
- 新增 `QUICK_TEST.md`：傻瓜式测试手册（5 个终端按序复制粘贴即可：driver/导航栈/al_ilqr/发点/录制，含急停、全部关闭、常见问题速查表、重新部署命令）
- 新增 `scripts/record_bag.sh`：al_ilqr_go2 专用 bag 录制脚本（在 NX 上跑）
  - 话题集：cmd_vel/status/optimized_path/odometry/scan/scan_360/initial_path/goal/arrive/map/tf/tf_static/rosout，可选 `--with-cloud` 追加原始点云
  - lz4 压缩 + 512MB 缓冲防丢帧，输出 `~/bags/YYYYMMDD_HHMMSS_<scene>.bag`
- 验证：脚本语法检查通过，已 rsync 到 NX（可执行权限保留），NX 上 rosbag 存在、磁盘余量 325G 充足
- 故障分析交付件约定：bag 文件 + 现象描述 + al_ilqr 终端告警截图
