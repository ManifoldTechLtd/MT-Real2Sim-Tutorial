# 从 MindCloudX 扫描数据生成 Isaac 仿真场景

完整端到端流程（Q9000 扫图 → XAI → USDZ → 地图 → 仿真 → ONS2 导航）见
[`end-to-end-scan-to-ons2.md`](end-to-end-scan-to-ons2.md)。转换脚本已随仓库提供在
[`mt-r2s/mindx2isaacsim/`](mt-r2s/mindx2isaacsim/)。

官方流程（公开教程）：

- 公开教程与源码：
  https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial/tree/master/docs/mt-r2s

## 1. 输入

| 文件 | 坐标系 | 用途 |
|---|---|---|
| `3dgs.ply`（平台下载 Z-up 版本） | Z-up | NuRec 3DGS 高保真视觉 |
| `mesh.ply` | Z-up、米制、三角面 | PhysX 碰撞体 |

目录约定：

```
my_scene/
├── ply/3dgs.ply
└── mesh/mesh.ply
```

## 2. 生成 USDZ（官方脚本）

```bash
# Python 3.10~3.12；依赖 numpy plyfile open3d usd-core msgpack scipy
python export_usdz.py \
  --ply_dir my_scene/ply \
  --mesh_dir my_scene/mesh \
  -o my_scene/usdz \
  --simplify_faces 500000 \
  --max_dist 0
```

输出 `scene.usdz`：`default.usda + scene.nurec + gauss.usda + mesh.ply + mesh.usd`，
包含 3DGS 视觉层和**不可见碰撞网格**（`collision_approx=none`，默认抽稀到 50 万面）。
PhysX 对超过约 500 万面的碰撞网格会静默失败；碰撞网格也不要少于约 50 面/m²。

本地注意：若没有安装 open3d，本仓库的 `add_mesh_to_usdz.py` 增加了等价的
顶点聚类抽稀回退；官方脚本本身未修改。

## 3. 接入 Real2Sim

1. 把 USDZ 放入 `dataset/`（不入 git）。
2. 在场景配置中设置：

```yaml
scene:
  environment:
    usdz_path: "/dataset/<scene>.usdz"
    position: [0, 0, -floor_z]       # 让选定的楼面落在 z=0；floor_z 见地图/点云统计
    add_collision: true
    keep_invisible_collision_meshes: true   # RTX可见，仅用hideForCamera隐藏相机主射线
  use_default_ground: false            # 碰撞网格有缺口时可改 true + ground_z_offset: 0
```

3. `robots/go2_sim` 提供仿真导航 profile，NeuPAN 直接复用 `robots/go2/neupan`。

## 4. 生成导航栅格地图

`tools/build_scan_scene.py` 支持两类扫描：

- 单层/室外：默认局部地面法（逐格最低点作为地面）。
- 多层室内：`--fixed-floor <z>` + `--free-min/--free-max`（地面高度带）+
  `--min-height/--max-height`（墙体/高障碍带）。示例（多层室内大堂）：

```bash
python tools/build_scan_scene.py --name floor2_hall \
  --mesh <mesh.ply> --pcd <cloud.pcd> --out-dir <out> \
  --resolution 0.05 --fixed-floor 1.67 \
  --min-height 1.2 --max-height 2.5 --free-min -0.2 --free-max 0.4
```

输出 `<name>_sim.pgm/.yaml` 和 `<name>_meta.json`（含 `floor_z`、`scene_position_z`、
`spawn_xy`）。2D 地图与 USDZ 共用同一世界坐标系，航点坐标可直接使用。

## 5. 已生成场景

| 场景 | USDZ | 地图 | 楼面 |
|---|---|---|---|
| 多层室内大堂 | `dataset/floor2_hall_3dgs.usdz` | `integrations/ons2/maps/floor2_hall_sim.*` | z≈1.67m |

注意：该扫描包含多层/大挑空，二层家具较多；多航点任务在局部地图与点云/碰撞不一致时
可能出现 `START_OCCUPIED`/`NeuPAN feedback stalled`，需要按具体区域清理/裁剪地图。
