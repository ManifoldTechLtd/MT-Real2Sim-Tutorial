[English](end-to-end-scan-to-ons2.md) | 简体中文

# 从 Q9000 扫图到 Isaac Sim 仿真导航

本教程用你自己扫描的场地制作 Isaac Sim 场景，并让 Go2 在场景中运行 ONS2 导航。
每一步都会说明 **输入** 和 **输出**，可以按顺序执行。

适用版本：Isaac Sim 5.1、ROS 2 Jazzy、ONS2、Go2 + ODIN1。

开始前先说明：场景文件和地图文件比较大，请放在自己的本地目录中。
本教程会用到 `MT-Real2Sim-Tutorial`（仿真）和 `Odin-Nav-Stack2`（导航）两个代码仓库。
所使用的 ONS2 版本必须包含 Go2 仿真配置；如果缺少，请先获取带仿真支持的版本。

## 0. 总览

```text
Q9000 扫描
  → 导出扫描工程
  → MindCloudX AI 训练
      ├─ cloud.pcd            → 导航地图 PGM/YAML
      ├─ 3dgs.ply + mesh.ply  → scene.usdz
      └─ map.bin              → 仅真机重定位使用
  → 配置场景和出生点
  → 启动 Isaac Sim + ONS2
  → 选航点，机器人导航
```

| 文件 | 用途 |
|---|---|
| `cloud.pcd` | 生成二维导航地图；可选生成颜色表 |
| `3dgs.ply` | 生成场景的画面（高斯视觉） |
| `mesh.ply` | 生成碰撞几何（地面、墙体） |
| `scene.usdz` | 放到 Isaac 场景中的最终文件 |
| `my_scene_sim.pgm/.yaml` | 导航使用的二维地图 |
| `my_scene_meta.json` | 楼层高度、建议出生点 |
| `my_scene_sim.npz` | 可选的点云回退颜色表 |
| `map.bin` | 真机 Odin 重定位地图，仿真不需要 |

下文用 `my_scene` 表示场景名，用 `my_scene_sim` 表示导航地图名。

## 1. 准备目录和终端

**输入：** 已安装 Docker 的开发电脑、两个代码仓库。

打开终端，按自己的实际路径修改并执行：

```bash
export SIM_ROOT=/path/to/MT-Real2Sim-Tutorial
export ONS2_ROOT=/path/to/Odin-Nav-Stack2
export SCENE_ROOT="$HOME/data/my_scene"
export DATASET_DIR="$SIM_ROOT/dataset"

export ROS_DOMAIN_ID=77

mkdir -p "$SCENE_ROOT/ply" "$SCENE_ROOT/mesh" "$SCENE_ROOT/out" "$DATASET_DIR"
cd "$SIM_ROOT"
```

后续命令默认都在这个终端执行。`DATASET_DIR` 是 Isaac 读取场景文件的目录。

**输出：** 准备好两个仓库路径和场景工作目录。

## 2. 用 Q9000 扫描场地

**输入：** Q9000、手机、MindCloud Go。

1. 手机安装 **MindCloud Go**。
2. 检查 Q9000 线缆和供电是否正常。
3. Q9000 开机，手机连接 Q9000 的 WiFi。名称通常包含设备序列号后四位，密码以设备说明为准。
4. 打开 App，点击 **连接设备**，等待连接成功。
5. 点击 **开始扫描**，再点绿色播放键，输入项目名 `my_scene`。
6. 手持Q9000缓慢走遍需要导航的区域。走道、门口和转角尽量多走几遍，扫描特征多的区域，最后回到已扫描区域（关于扫描手法可以向留形科技售后FAE询问）。
7. 点击停止，确认项目已保存。
8. 用 USB 3.0 连接 Q9000 数据口和电脑，或使用设备的导出功能，把本次扫描工程完整复制到电脑。

扫描时不要走得太快，避免拖影和漏扫；玻璃、镜面和反光地面容易产生错误点，后续要检查地图。

**输出：** 电脑上的完整扫描工程文件夹。

## 3. 在 MindCloudX AI 训练并下载

**输入：** 第 2 步得到的完整扫描工程。

1. 打开 https://mindcloudx.ai/，登录或注册。
2. 点击 **上传数据集**，选择本次扫描工程文件夹并上传。
   建议先把工程复制到电脑硬盘再上传，避免 U 盘或设备断连。
3. 上传完成后进入同名项目。我们需要 **点云、3DGS、Mesh**：
   在对应页签分别创建训练任务。
4. 训练参数保持默认，按需开启“动态物体滤除”，等待完成并预览结果。
5. 下载文件：

```text
my_scene/
├── cloud.pcd
├── ply/
│   └── 3dgs.ply      # 下载 Z-up 版本
└── mesh/
    └── mesh.ply
```

6. 如果还要在**真机**上运行导航，另外下载同项目的 `map.bin`；只做仿真时不需要。

确认三个文件来自同一项目；文件名不同时可以重命名，并按下面的位置放好：

```bash
cp /path/to/cloud.pcd "$SCENE_ROOT/cloud.pcd"
cp /path/to/3dgs.ply  "$SCENE_ROOT/ply/3dgs.ply"
cp /path/to/mesh.ply  "$SCENE_ROOT/mesh/mesh.ply"
```

**输出：** `cloud.pcd`、`ply/3dgs.ply`、`mesh/mesh.ply`，以及可选的 `map.bin`。

## 4. 把 3DGS 和 Mesh 转成 USDZ

**输入：** `ply/3dgs.ply`、`mesh/mesh.ply`。
如果已经有可用的 USDZ，可以跳过本步，直接执行第 5 步生成地图。

转换脚本在 `docs/mt-r2s/mindx2isaacsim/`。先安装依赖：

```bash
cd "$SIM_ROOT"
python3 -m venv .venv-scene
source .venv-scene/bin/activate
pip install -r docs/mt-r2s/mindx2isaacsim/requirements.txt
pip install pyyaml python-lzf
```

需要使用 Python 3.10–3.12。若系统提示缺少 `venv`，先安装对应的 `python3-venv` 软件包。

执行转换：

```bash
python docs/mt-r2s/mindx2isaacsim/export_usdz.py \
  --ply_dir "$SCENE_ROOT/ply" \
  --mesh_dir "$SCENE_ROOT/mesh" \
  -o "$SCENE_ROOT/usdz" \
  --simplify_faces 500000 \
  --max_dist 200

cp "$SCENE_ROOT/usdz/scene.usdz" "$DATASET_DIR/my_scene.usdz"
```

参数说明：

| 参数 | 含义 | 使用建议 |
|---|---|---|
| `--ply_dir` | 3DGS PLY 所在目录 | 放 `3dgs.ply` |
| `--mesh_dir` | Mesh PLY 所在目录 | 放 `mesh.ply` |
| `-o` | USDZ 输出目录 | 生成 `scene.usdz` |
| `--simplify_faces` | 碰撞网格抽稀到多少面 | 默认 50 万；过少可能漏碰撞，过多可能导入失败 |
| `--max_dist` | 丢弃离原点多远的高斯 | 默认 200 m；`0` 表示不过滤 |
| `--mesh_visible` | 碰撞网格是否可见 | 默认不可见，只用于物理和雷达 |

碰撞网格默认使用 `collision_approx=none`，不要修改。
Z-up 数据不要加 `--y_up`，该选项只用于旧版 Y-up 数据。

**输出：** `$DATASET_DIR/my_scene.usdz`。
这个文件负责画面和碰撞；导出成功不代表碰撞正确，启动后仍需检查机器人不会穿地或穿墙。

## 5. 生成导航地图（重点）

**输入：** 同一场地的 `cloud.pcd`、`mesh.ply`。

ONS2导航需要 PGM/YAML 二维地图，不能直接用 USDZ。这里使用项目自带的地图工具。
如果第 4 步已经生成 USDZ，一定要加 `--skip-usdz`，否则工具会再生成一个只带网格的 USDZ。
如果跳过了第 4 步，请先按第 4 步安装依赖并执行 `source "$SIM_ROOT/.venv-scene/bin/activate"`。

```bash
cd "$SIM_ROOT"
source .venv-scene/bin/activate

python tools/build_scan_scene.py \
  --name my_scene \
  --mesh "$SCENE_ROOT/mesh/mesh.ply" \
  --pcd "$SCENE_ROOT/cloud.pcd" \
  --out-dir "$SCENE_ROOT/out" \
  --skip-usdz \
  --resolution 0.05

cp "$SCENE_ROOT/out/my_scene_sim.pgm" \
   "$SCENE_ROOT/out/my_scene_sim.yaml" integrations/ons2/maps/
```

### 5.1 参数含义

| 参数 | 含义 |
|---|---|
| `--name` | 场景名。`--name my_scene` 会输出 `my_scene_sim.pgm/.yaml` 和 `my_scene_meta.json` |
| `--mesh` | Mesh 文件。命令必须提供；加 `--skip-usdz` 时不会重做 USDZ，但仍需传入以满足命令格式 |
| `--pcd` | 扫描点云，用于生成地图 |
| `--out-dir` | 输出目录 |
| `--skip-usdz` | 只生成地图；已有 USDZ 时使用 |
| `--resolution` | 每格多少米。`0.05` 表示 5 cm；地图很大时可改 `0.1` |
| `--fixed-floor` | 指定要导航的楼层在原始点云中的 Z 高度；不填时取低处约 2% 的点作为地面 |
| `--min-height` / `--max-height` | 相对楼面的障碍物高度范围，默认 `0.30 / 0.90` 米 |
| `--free-min` / `--free-max` | 相对楼面的可通行高度范围，默认 `-0.30 / 3.00` 米 |
| `--color-voxel`、`--color-stride`、`--color-max-points` | 只在自动生成网格 USDZ 时使用，`--skip-usdz` 时不需要 |

**`--fixed-floor` 怎么定：**
单层场地先用默认值生成地图，打开 PGM 检查；如果地面判断正确，就不需要这个参数。
多层或挑空场地，先看 `my_scene_meta.json` 里的 `floor_z`，或者用点云查看工具找到
准备让机器人行驶的楼面高度，再把这个 Z 值填给 `--fixed-floor`。
其余 `--min-height/--max-height` 和 `--free-min/--free-max` 都相对于这个楼面，
按现场障碍物和楼板厚度调整；不要把其他楼层投到当前楼层。

### 5.2 工具输出

以 `--name my_scene` 为例：

- `my_scene_sim.pgm`：白=可通行，黑=障碍，灰=未知；
- `my_scene_sim.yaml`：地图分辨率和原点；
- `my_scene_meta.json`：`floor_z`、`scene_position_z`、`spawn_xy` 等。

地图和 USDZ 使用同一套 X/Y 坐标。若场景在 X/Y 方向平移，地图、出生点、航点必须一起平移，
不能只移动 USDZ。

### 5.3 默认地面、楼层高度和出生点

这是一个容易出错的点，先分清三个高度：

1. **默认地面**：Isaac 中一块水平的兜底地面，位置是世界的 `z=0` 加上
   `scene.ground_z_offset`（扫描场景常用 `-0.02`）。它不是扫描出来的楼面高度。
2. **场景楼层**：扫描点云中的楼面。地图工具会给出 `floor_z` 和建议的
   `scene_position_z`，其中 `scene_position_z = -floor_z`。
3. **机器人出生高度**：机器人初始位置。它不是楼面高度，而是楼面上方的**下落高度**。

正确做法：

```text
把 scene.environment.position.z 设为 scene_position_z
    → 扫描楼面落到 z=0
把 quadruped.position 的 X/Y 设为 spawn_xy
    → 机器人站在地图空白区域
把 quadruped.position.z 设为 0.5 左右
    → 机器人从楼面上方落下一点，运控能站住
```

举例：如果 meta 的 `floor_z` 是 `2.00`，就设 `scene_position_z=-2.00`，
场景配置的 `position.z` 填 `-2.00`，出生 `position.z` 仍填 `0.5`。
如果 `floor_z` 是 `-0.50`，就填 `+0.50`。关键是让扫描楼面到 `z=0`，
不是让出生高度去抵消楼层高度。

配置示例：

```yaml
scene:
  environment:
    position: [0.0, 0.0, 0.0]       # 改成 meta 的 scene_position_z
  use_default_ground: true         # 扫描网格有缺口时兜底
  ground_z_offset: -0.02
  physics:
    collision_approximation: none

quadruped:
  position: [0.0, 0.0, 0.5]        # X/Y 改成 meta 的 spawn_xy；Z 是下落高度
```

- **出生点不要用很高**：0.4–0.6 m 通常足够让 Go2 落下并站住；
  从 1 m 以上落下可能弹跳、翻倒或损坏。
- **不要出生在地板里面**：如果 Z 比楼面还低，机器人可能被弹开或穿模。
- **不要靠拖动 USDZ 猜楼层**：在 GUI 里拖动只适合粗略查看，精确位置请改
  `scene.environment.position` 的数字。拖动后地图和航点不会自动跟着变。
- **没有 `my_scene_meta.json` 怎么办**：用点云查看工具或地图工具找到楼面 Z；
  在 Isaac 里也可以放一个临时方块/标记到楼面，读取它的 Z 值。
  不要用机器人的出生高度反推楼面高度。
- **如何验证**：启动后等机器人稳定，读取 `/odin1/odometry` 或
  `/state_estimation` 的 z。机器人应稳定在楼面上方约 0.3–0.5 m；
  如果 z 持续下降，说明碰撞地面缺失或出生在空洞；
  如果一开始就穿地或弹飞，检查 `scene_position_z` 和出生 Z。
- **默认地面什么时候用**：扫描网格没有完整地面、局部有洞时，`use_default_ground: true`
  会在 `ground_z_offset` 放一块水平兜底地面。多层、坡道或大范围场景建议关闭它，
  改用 USDZ 的碰撞网格，避免兜底地面把真实低处/高处挡住。

**输出：** `integrations/ons2/maps/my_scene_sim.pgm` 和 `.yaml`，
以及 `my_scene_meta.json` 中的楼层/出生点信息。

## 6. 配置 Isaac 场景

**输入：** `my_scene.usdz`、`my_scene_sim.pgm/.yaml`、`my_scene_meta.json`。

复制通用 Go2 配置作为模板：

```bash
cd "$SIM_ROOT"
cp r2s2r/configs/go2.yaml r2s2r/configs/go2_my_scene.yaml
```

打开 `go2_my_scene.yaml`，修改以下字段；其他字段先保持模板值：

```yaml
simulation:
  headless: false
  window_width: 1280
  window_height: 720

scene:
  environment:
    enabled: true
    usdz_path: "/dataset/my_scene.usdz"   # 容器内路径，不是电脑上的绝对路径
    position: [0.0, 0.0, 0.0]            # 改成 meta 的 scene_position_z
    add_collision: true
    keep_invisible_collision_meshes: true
  use_default_ground: true
  ground_z_offset: -0.02
  physics:
    collision_approximation: none

quadruped:
  robot_type: go2
  prim_path: /World/Quadruped
  position: [0.0, 0.0, 0.5]               # X/Y 改成 spawn_xy；Z 是下落高度

sensor_suite:
  enabled: true
  calibration_dir: calibration/odin1_go2
  camera_render_scale: 0.5

bridge:
  pointcloud_rate: 10
  image_rate: 5
  image_quality: 70
```

注意：

- `/dataset/my_scene.usdz` 是容器内路径，对应电脑上的 `DATASET_DIR/my_scene.usdz`；
- `scene_position_z` 来自 `my_scene_meta.json`，不是出生 Z；
- 出生 X/Y 用 `spawn_xy`，并确认在地图白色区域内、离障碍物至少约 1 m；
- 出生 Z 用 0.5 左右，让机器人落下站住；
- 如果场景没有完整地面，保留 `use_default_ground: true` 和 `ground_z_offset: -0.02`；
- 机器人站立高度、相机频率和分辨率可以按机器性能和画质需要调整。

**输出：** `r2s2r/configs/go2_my_scene.yaml`。

补充：后续我们会添加璇玑动力的sw01模型和宇树科技的As2和运控用于仿真训练。此外仓库中也提供了一个简易的长方体差速车模型，用于验证针对差速车的算法。

## 7. 生成点云颜色表（可选）

**输入：** 带颜色的 `cloud.pcd`、第 6 步 `scene.environment.position` 的 X/Y/Z 平移量。

非必须，开启后可以获得更好的着色观感。

```bash
cd "$SIM_ROOT"
source .venv-scene/bin/activate

python tools/build_color_lut.py \
  --pcd "$SCENE_ROOT/cloud.pcd" \
  --out integrations/ons2/color_luts/my_scene_sim.npz \
  --voxel 0.1 --stride 2 --max-points 25000000 \
  --translation 0 0 0.0
```

`--translation` 必须和第 6 步 `scene.environment.position` 的 X/Y/Z 完全一致。
示例里的 `0 0 0.0` 只是占位；Z 用 meta 的 `scene_position_z`，X/Y 也平移过就填对应的 X/Y。
文件名要和地图名 `my_scene_sim` 一致。
不生成这个文件也可以运行，只是相机没有覆盖的点会使用默认颜色。

**输出：** `integrations/ons2/color_luts/my_scene_sim.npz`。

## 8. 构建运行环境并编译导航

**输入：** 两个代码仓库、Docker、NVIDIA GPU、场景配置。

构建镜像：

```bash
cd "$SIM_ROOT"
docker build -f docker/Dockerfile.isaac -t r2s2r-isaac:5.1.0 .
docker compose -f docker/docker-compose.yml build ros2
```

如果拉取 Isaac 基础镜像需要登录，请按 NVIDIA NGC 的说明登录终端；不要把账号或令牌写进仓库。

启动 ROS2 容器并编译导航工作区：

```bash
cd "$SIM_ROOT"
ONS2_ROOT="$ONS2_ROOT" DATASET_DIR="$DATASET_DIR" \
  docker compose -f docker/docker-compose.yml up -d ros2

docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /ons2/ros2_ws &&
  colcon build --symlink-install --packages-select \
    nav_interfaces atomic_move map_planner sliding_map neupan_ros mission_manager \
    ons2_bringup sim_bridge_ros2'
```

**输出：** 两个镜像和导航工作区编译结果；编译报错时先处理，不要继续启动。

## 9. 启动仿真、选择航点并验收

**输入：** 第 6 步的场景配置、第 5 步的地图、第 8 步的编译结果。

```bash
cd "$SIM_ROOT"
SIM_CONFIG=r2s2r/configs/go2_my_scene.yaml \
DATASET_DIR="$DATASET_DIR" \
./start.sh --robot go2 --runtime ros2 --stack ons2 \
  --map my_scene_sim \
  --ons2-root "$ONS2_ROOT"
```

等待 Isaac 窗口加载场景。如果窗口处于暂停状态，点击 **PLAY**。
启动完成后，导航控制台在容器 tmux 中：

```bash
docker exec -it isaac_nav_ros2 tmux attach -t ons2_sim
```

### 选航点

在 Foxglove 连接本机 8765 端口，打开 3D 面板：

1. 可视化地图 `/map` ，点云 `/odin1/cloud_slam`，图像`/odin1/image/compressed`以及其他您需要的话题；
2. 发布话题设为 **`/waypoint_pick`**，坐标系设为 **`map`**（和ONS2保持一致）；
3. 使用 Pose 工具在地图白色区域选点；
4. 在控制台创建单点或多点任务并启动。

也可以使用预设航点。新建 `integrations/ons2/waypoints/my_scene_sim.yaml`：

```yaml
waypoints:
  - {name: P1, x: 1.0, y: 0.0, yaw: 0.0, dwell_sec: 2.0}
  - {name: P2, x: 2.0, y: 1.0, yaw: 1.57, dwell_sec: 2.0}
```

坐标只是格式示例，必须换成自己地图中的安全位置；`yaw` 用弧度，`dwell_sec` 是停留秒数。
发送任务：

```bash
bash integrations/ons2/waypoints/send_waypoints.sh \
  --runtime ros2 --scene my_scene_sim
```

### 点云颜色

展开 `/odin1/cloud_slam` 设置：

- 颜色模式选 **BGR（打包）**；
- 颜色字段选 **`rgb`**，不要保留“距离（自动）”。

### 相机视角与录屏

视口左上角相机图标 → **Cameras** 里保留机器狗的前视传感器相机 **camera1**。
它跟随头部，输出 `/odin1/image/compressed`（800×648），也是点云着色使用的图像；
这是正常跟狗移动的视角。

录屏时选 **camera1** 获取狗的前视画面，地图和点云可放在另一个面板。

`camera1` 是着色用的传感器，不影响导航或航点。

### 验证

```bash
docker exec -it isaac_nav_ros2 bash
source /opt/ros/jazzy/setup.bash
source /ons2/ros2_ws/install/setup.bash
ros2 topic echo /waypoint_manager/status --once
```

应能看到任务从 `NAVIGATING`、`DWELLING` 到 `COMPLETED`。
图像、点云和地图应持续更新；仿真变慢时，实际频率可能低于配置值。

停止：

```bash
cd "$SIM_ROOT"
./start.sh --stop
```

**输出：** 机器人沿航点移动，任务完成。

## 10. 真机导航和仿真的区别

本教程以仿真为主。如果还要用同一扫描数据在真机上导航，需要额外准备：

| | 仿真 | 真机 |
|---|---|---|
| 场景 | USDZ | 真实场地 |
| 定位 | 仿真位姿 | Odin 定位 |
| 二维地图 | PGM/YAML | PGM/YAML |
| 重定位地图 `.bin` | 不需要 | 按 Odin 驱动要求配置 |
| 控制 | 仿真 `/cmd_vel` | 真实底盘 |

真机转换 PCD 到 PGM/YAML 时，使用导航仓库中的 `pcd_to_pgm.py`，
并按真实安装高度设置雷达高度和切片范围。PCD、PGM、YAML、BIN 需要同名，
YAML 内的 `image` 字段也要对应。真机出生/定位位置不能直接使用仿真出生点。

## 11. 常见问题

| 现象 | 检查 |
|---|---|
| 找不到 `go2_sim` 仿真配置 | 使用的 ONS2 版本是否包含 Go2 仿真支持 |
| `docker exec` 找不到容器 | 先执行第 8 步启动 ROS2 容器 |
| 找不到 USDZ | `DATASET_DIR` 是否正确；配置里是否使用容器内 `/dataset/` 路径 |
| 地图和点云错位 | 是否来自同一项目；场景、地图、出生点的 X/Y 是否一起处理 |
| 起点不可用、`START_OCCUPIED` | 出生点是否在白色区域，离障碍物是否足够远 |
| 机器人出生后穿地/弹飞 | 检查 `scene_position_z`、出生 Z 和碰撞地面；不要只调出生高度 |
| 机器人穿透地面 | 3dgs生成的碰撞网格是否有洞，面数是否合适 |
| 仿真很慢 | 降低相机比例、图像频率或 GUI 分辨率；不要用提高速度掩盖卡顿 |
| TF 过期或位姿跳变 | 导航节点使用仿真时钟，避免和其他 ROS2 系统共用 domain |


与ONS2的集成可参考 [ONS2 集成目录](../integrations/ons2/README.md)。

## 12. 参考

- 官方教程与脚本：
  https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial/tree/master/docs/mt-r2s
- ONS2 导航文档：`Odin-Nav-Stack2/docs/simulation/04_simulation.md` / `.zh-CN.md`

> 请只使用自己有权使用的扫描数据，并遵守场地所有方和数据提供方的授权要求。
