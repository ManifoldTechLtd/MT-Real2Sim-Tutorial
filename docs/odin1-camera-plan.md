# Odin1 相机一致性与底部黑区排查计划

**状态：阶段 A 黑区已定位并修复（2026-11-25）；阶段 B 鱼眼待评审实施。** 没有修改镜头模型或颜色 LUT，没有提交或推送。

**目标：**先定位针孔相机图像底部黑色三角区，再使仿真鱼眼渲染、点云着色和去畸变使用同一套 Odin1 标定。

**架构：**图像异常排查与鱼眼改造分两阶段。先在原针孔模型下建立同帧 RGB/JPEG 与场景隔离证据；排除异常后，用完整七阶 FishPoly 数学模型生成镜头方向纹理，并让桥接投影和去畸变复用该模型。

**技术：**Isaac Sim 5.1、USD、RTX/NuRec、NumPy、OpenEXR、ROS1/ROS2、OpenCV remap。

**评审依据：**用户要求保留当前颜色 LUT、后续再实施鱼眼；模型参考为文末飞书文档。本文不声称目前已经复刻真实 Odin1。

## 1. 约束与执行顺序

- 保留 GUI，不修改 GPU 驱动，不停止或改动真机导航栈。
- 当前颜色 LUT 文件、gamma 和回退策略保持不变。
- 镜头映射纹理与颜色 LUT 是两类不同数据，不混用目录或参数。
- 不以裁掉图像底部、提高相机安装位置或改外参掩盖黑区。
- 不把标定系数写成固定常数；以设备标定文件为来源。
- 每次诊断只改变一个因素；保存前后画面、属性和位姿，恢复原状态。
- 黑区原因明确且验证通过后，才开始鱼眼改造。
- 默认只本地验证；提交、推送、合并单独授权。

## 2. 底部黑区：目前证据与不确定项

### 2.1 已确认

1. 用户截图出现底部黑带及灰白三角形。直接订阅 `/odin1/image/compressed` 保存的当前 JPEG 也有同类伪影，因此不是 Foxglove 面板、颜色模式或显示缩放造成。
2. 当前相机 `intrinsics_camera1.yaml` 仍是 `model: pinhole`；未启用 FishPoly 镜头纹理，这次伪影不能归因于鱼眼畸变算法。
3. `core/observation.py` 对 RGB 做 uint8/HWC 校验和 C 连续复制；`bridge/protocol.py` 通过 OpenCV 编码 JPEG；ROS2 桥接原样发布 JPEG。未发现手工字节步长或裁切代码，但尚未保存同一帧的原始 RGB，所以编码前异常还需实测。
4. `sensor_suite.py` 创建相机时没有设置 `clippingRange`。容器内 USD Camera 的默认属性为 `(1, 1000000)`；项目使用米制场景。此值是源码/fallback 检查，尚未从运行中 Stage 读取实际属性。
5. 园区场景开启默认地面。`add_default_ground_plane()` 会加载 Grid 环境，不能假设它仅是不可见的物理平面。
6. 环境原本不可见的碰撞代理被恢复可见，并设 `primvars:hideForCamera`。该逻辑不覆盖默认地面和 Go2 视觉网格。
7. Go2 相机位姿按当前外参计算约为 body 中 `[0.323187, 0.006461, 0.103574]` 米。它接近机器人视觉几何，但包围盒不能证明射线与机器人发生相交。
8. `run_sim.py` 当前没有应用 YAML 中的 `simulation.render_opt` 参数；不能声称反射、阴影、透明渲染实际都关闭了。
9. 园区 NuRec 模型自己的近裁剪为 `1e-8`，与 USD 相机默认近裁剪是不同层级。二者最终如何组合尚未由运行时证据确定。

### 2.2 候选原因（near 已确认，其余待验证）

| 假设 | 为什么值得检查 | 证伪方法 |
|---|---|---|
| **相机近裁剪（已确认）** | 低安装相机未设置 near；普通几何与高斯裁剪层不同 | 已修复：near 1m→0.05；见 2.3 |
| 默认地面或碰撞代理进入图像 | 额外地面在运行场景中；代理使用相机专用隐藏属性 | 分别隔离默认地面、代理 mesh 的相机可见性 |
| Go2 自遮挡/相机靠近视觉 mesh | 相机在头部近处，截图伪影位于底部 | 仅隐藏机器人 visuals，其他属性不变 |
| NuRec 近场 splat 或混合透明合成异常 | 黑区边缘呈不规则三角形 | 排除普通几何后仍复现，再比较 NuRec-only/mesh-only |
| RGB readback/JPEG 路径异常 | 目前只证实 JPEG 有问题 | 同一帧原始 RGB 与解码 JPEG 比较 |

“在 GUI 中看起来正常”不足以排除场景渲染：GUI 与传感器相机的位姿、near 和 render product 可以不同。必须使用同一相机、同一位姿比较。

### 2.3 执行记录（2026-11-25）

- 运行时读取：相机 `clippingRange` 初始为 `(1.0, 1000000.0)`，此前创建链路从未设置。
- 同一场景/位姿把裁剪改为 `near=0.05, far=1000` 后，图像底部黑带和灰白三角消失，近地面正常。
- 同帧证据：相机 Writer 保存的 800×648 原始 PNG、同数组 JPEG、ROS `/odin1/image/compressed` 均为干净画面；排除 Foxglove、JPEG 编码和颜色 LUT。
- 根因：USD Camera 默认近裁剪 1 m，低安装相机把近处地面/3DGS 边界裁掉，形成黑带；灰白三角是裁剪边界处的地面/高斯边缘。
- 修复：`SensorSuite` 默认 `camera_clip_near=0.05, camera_clip_far=1000`，可被 `sensor_suite.camera_clip_near/far` 覆盖；启动日志打印 before/after。未裁图、未改相机位姿、未动 LUT。
- 说明：该 A/B 同时调整 near/far；远裁剪 1000 不会解释近处底部黑带。若需严格单变量，可保持 near=0.05、far=原值再测一次。
- 未做：默认地面/机器人/代理隔离 A3（当前症状已由 near 解释）；二楼/办公回归、GUI 同相机检查、鱼眼 B 阶段待做。

## 3. 阶段 A：黑区诊断，先执行这一部分

### A1. 建立固定条件基线

涉及：`r2s2r/core/sensor_suite.py`、`r2s2r/core/observation.py`、`r2s2r/run_sim.py`；诊断输出放系统临时目录，不进入源码只读挂载。

- [ ] 确认导航任务停止、速度为零；记录场景、机器人与相机世界位姿、800×648 分辨率、完整相机属性和当前有效 renderer 设置。
- [ ] 在相机 Writer 交付时保存一帧原始 RGB PNG，并把相同数组编码为 JPEG；同时保存 capture timestamp，不用两次不同订阅冒充同帧。
- [ ] 比较 PNG、JPEG 和 ROS JPEG：原始 PNG 已出现同样黑区，则定位到采集/渲染之前；仅 JPEG 出现则调查编码边界。
- [ ] 将 GUI viewport 切到该传感器相机，而不是自由视角；确认 GUI 与独立 render product 是否同时出现伪影。

### A2. 近裁剪单变量 A/B

涉及：`sensor_suite.py` 相机创建路径；Camera API `get_clipping_range()` / `set_clipping_range()`。

- [ ] 读取实际 `clippingRange`，不可直接把 fallback 当现场属性。
- [ ] 保持 far、位姿、场景、光照不变，临时比较 near 当前值、0.1 m、0.01 m；不使用 0（API 实现使用 truthy 判断）。
- [ ] 每组等待相同渲染稳定帧数，记录黑区高度、三角边界和普通物体是否被裁掉。
- [ ] 如果 near 改变能消除异常，仍需区分正常裁剪与 NuRec/普通地面混合问题，不能直接把参数改动视为根因证明。
- [ ] 对照结束恢复原 near。

### A3. 几何隔离 A/B

涉及：`core/scene_builder.py`、默认 GroundPlane、机器人 visuals、Environment 代理。

- [ ] 仅隐藏默认地面可渲染部分的相机主射线，保留碰撞；保存前后帧，恢复。
- [ ] 仅隐藏 Go2 的 visuals，保留 body、传感器、碰撞；保存前后帧，恢复。
- [ ] 仅隔离 Environment 的碰撞代理，检查 `hideForCamera` 在该渲染路径是否生效；恢复。诊断期间任何影响 RTX 雷达的变更不得用于导航运动测试。
- [ ] 将场景限制为 NuRec-only/mesh-only，保持相机世界位姿不变；必要时再比较 near。
- [ ] 在简单平面+立方体场景用相同相机/Writer做对照；若简单场景也出现，优先查相机/readback，若只在 NuRec 出现则优先查混合渲染。

### A4. 确认根因后的最小修复

- [ ] 根据 A1–A3 的结果选择唯一修复点：裁剪配置、可见性、相机/模型安装、编码，或 NuRec 合成配置。
- [ ] 对该修复编写能够失败的回归测试，再实现最小改动；不同时改鱼眼或 LUT。
- [ ] 园区与大厅均验证静止、转向、近地面、靠墙画面；检查 GUI、雷达障碍回波、图像时间戳没有回归。
- [ ] 汇报根因证据和未解决限制，再评审是否进入阶段 B。

## 4. 阶段 B：Odin1 FishPoly 鱼眼改造

### B0. 补齐设备资料

- [ ] 当前 Odin1 完整标定：原始分辨率、A11/A12/A22/u0/v0、k2–k7、各传感器外参、单位/轴定义、裁剪和旋转规则。
- [ ] 确認实际有效视场及像素遮罩；明确 `maxIncidentAngle` 是模型定义域还是物理镜头可见角。不能直接把 120°写成整幅图像240°视场。
- [ ] 固定 SDK 版本和 `polynomial_camera.hpp`、`buildUndistortMap()` 实现，取得去畸变目标 K、输出尺寸和 ROI。
- [ ] 验证数据：原始压缩图、去畸变图、cloud、odom、TF，含静止标定板和缓慢旋转。
- [ ] 快门类型、曝光/读出时间和相机雷达时间偏差；没有这些只能声称几何模型对齐。

### B1. 共享 FishPoly 数学模型

涉及：`core/fishpoly_lut.py` 的七阶径向函数与数值逆解、ROS1/ROS2 的投影调用。

- [ ] 建立同一个相机模型入口：完整仿射内参 + 七阶径向多项式 + 有效域。
- [ ] 正投影使用 ray→pixel；反投影使用 pixel→ray，包含 skew、非中心主点和轴转换。
- [ ] 数值求逆检查径向单调性、合法根与残差，不用钳位到最大角后仍把像素当有效。
- [ ] 测试光轴、四个边角、非零 skew、非中心主点、边界无效像素、正反投影往返和800×648缩放。
- [ ] 与固定 SDK 实现逐点比较；初步目标为全分辨率有效域内像素误差≤1px，最终阈值结合真实标定残差确定。

### B2. 原生镜头映射纹理

涉及：`core/fishpoly_lut.py`、`core/sensor_suite.py`；新依赖 OpenEXR 需进入镜像依赖清单。

- [ ] 按完整 FishPoly 生成 direction/position 两张 EXR，接 `OmniLensDistortionLutAPI`，不继续使用七阶到四阶的降维拟合。
- [ ] 核对 Isaac 5.1 实际 API：视空间 -Z 前、纹理像素中心采样、无效光线哨兵、八面体编码的 Z 翻转。
- [ ] 非中心主点仅应用一次；主点若已烘焙到纹理，不再通过镜头属性重复偏移。
- [ ] 在系统缓存目录写纹理与诊断，缓存键包括完整标定和输出尺寸，不能写到容器只读源码目录。
- [ ] 单独验证标定板渲染与数学投影，检查时域累计/运动矢量，不用静止画面正常替代运动验证。
- [ ] 保留 GUI、0.5 分辨率配置可调，不承诺物理噪声/ISP/曝光已复刻。

### B3. 图像元数据和点云着色

涉及：`r2s2r/bridge/protocol.py`、`r2s2r/run_sim.py`、`ros2_ws/src/sim_bridge_ros2/sim_bridge_ros2/camera_projection.py`、ROS1 bridge。

- [ ] 图像消息传模型名、完整标定、实际尺寸、有效域、body→camera和采集时间。
- [ ] 三维点变到图像时刻相机坐标后按 FishPoly 正投影取色；不再使用 pinhole 的除深度公式。
- [ ] 鱼眼有效域允许超过90°的入射角时，不沿用针孔的 `X>0` 筛选；遮挡比较采用同射线距离/一致定义，不误用针孔轴向深度。
- [ ] 检查快速转向时的图像延迟、姿态插值/最近样本误差；静态与动态条件分别报告。
- [ ] 保留当前体素颜色 LUT、gamma及回退策略，不重新生成颜色表，不改其参数。
- [ ] ROS1/ROS2 共用模型计算，不在一个端实现后声称双端完成。

### B4. 与真机一致的去畸变话题

涉及：ROS1/ROS2 bridge 的图像发布路径。

- [ ] `/odin1/image/compressed` 发布原始鱼眼 JPEG；不暗中换成去畸变图。
- [ ] 使用 SDK 同款目标 K/尺寸建反向映射表，双线性 remap 输出 `/odin1/image/undistorted`。
- [ ] 如发布 CameraInfo，明确它使用的畸变模型是否为标准 ROS 模型；非标准七阶系数不能冒充 OpenCV equidistant 四系数。
- [ ] 点云着色明确使用原始鱼眼图及对应模型，避免把去畸变图搭配原始参数。
- [ ] 验证直线恢复、裁剪、话题编码、时间戳、图像与雷达对齐及导航回归。

## 5. 阶段 C：Odin1 STP 外壳与安装

这一阶段不阻塞镜头模型验证。

- [ ] 向机械/硬件团队索取对应版本的 STP/STEP：外壳、镜头/雷达窗口、安装孔和Go2支架。
- [ ] 同时取得 CAD 基准到相机光心、雷达测量原点、IMU 原点的变换，以及质量/质心/惯量。
- [ ] 明确开源再分发授权；不能因有内部 CAD 文件就把模型上传 GitHub。
- [ ] 转为 USD：视觉网格用于外观和自遮挡，简化碰撞体用于 PhysX；传感器安装在标定测量原点而非外壳中心。
- [ ] 验证安装间隙、相机视场、雷达自遮挡与自滤规则。STP 不能代替镜头标定或雷达反射/固件模型。

## 6. 交付与验收

- 黑区：有同帧原始 RGB/JPEG、实际相机属性、A/B 记录，定位到具体模块；两个场景无原症状。
- 鱼眼：完整标定可配置，SDK 对照/往返测试、渲染标定板与运动测试通过。
- 着色：鱼眼几何与图像一致，LUT维持原状；时间偏差和遮挡近似明确。
- 话题：原始鱼眼与去畸变分开，双端状态分别说明。
- 性能：保留 GUI，对比原针孔/鱼眼 GPU占用、帧率、图像频率和仿真时间比。
- 发布：先本地审阅，再决定分支、提交、推送；不顺带修改导航正式分支。

## 7. 参考

- Odin1/Q9000 鱼眼模型：https://vvcazjv268.feishu.cn/wiki/Mx2OwzGdZiOYQUkbVW2cEbp8nAg
- NVIDIA 镜头映射：https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html
- 本仓库：`r2s2r/core/sensor_suite.py`、`r2s2r/core/fishpoly_lut.py`、`r2s2r/core/observation.py`、`r2s2r/core/scene_builder.py`、`r2s2r/core/robot_factory.py`、`r2s2r/bridge/protocol.py`、ROS bridge。

阶段 A 已执行：近裁剪 confirmed，修复已落到默认值；阶段 B 鱼眼与 STP 尚未开始。
