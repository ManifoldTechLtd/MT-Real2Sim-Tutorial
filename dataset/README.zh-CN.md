[English](README.md) | 简体中文

# 演示数据集

本目录是仿真器读取场景文件的目录（`DATASET_DIR`）。Docker 会把它以只读方式挂载到容器内的
`/dataset`，场景配置中也使用 `/dataset/<scene>.usdz` 这样的容器内路径引用文件。

## 当前状态

git 中只跟踪本 README（以及 `README.zh-CN.md` / `.gitkeep` 占位文件），**仓库不包含任何扫描数据**。
我们正在为“零门槛首次运行”准备一个小型演示环境，它会和其他大体积资产一起，在 git 之外单独提供。

大型扫描数据、USDZ 场景和颜色表（LUT）资产都不纳入 git，由发布包或项目公布的数据渠道单独提供。
如果目录中没有任何场景文件，仿真仍然可以启动，但环境是空的，没有可导航的场地。

## 预期目录结构

演示数据准备就绪后，目录应如下所示：

```text
dataset/
├── README.md
├── README.zh-CN.md
├── .gitkeep
└── <scene>.usdz        # Isaac 场景：3DGS 画面 + 碰撞网格
```

- 本目录只消费 `<scene>.usdz` 这一种资产。文件名要与所选场景配置中的期望值一致；现有配置使用的
  文件名包括 `office_0923.usdz`、`floor2_hall_3dgs.usdz`、`park_inspection_3dgs.usdz`，
  默认配置使用 `tunnel1.usdz`。
- 二维地图、航点任务和颜色表**不放在这里**，见下表。
- 除 README 外，不要提交本目录中的文件。

| 资产 | 位置 | 是否纳入 git |
|---|---|---|
| USDZ 场景（画面 + 碰撞） | `dataset/` | 否 |
| 二维导航地图（PGM/YAML） | `integrations/ons2/maps/` | 是（小体积地图） |
| 预设航点任务 | `integrations/ons2/waypoints/` | 是 |
| 颜色表（`.npz`） | `integrations/ons2/color_luts/` | 否 |

## 首次运行

在仓库根目录执行：

```bash
cd /path/to/MT-Real2Sim-ons2
export DATASET_DIR=$PWD/dataset
./start.sh --robot go2
```

同一启动方式的其它写法：

```bash
# 使用默认配置启动 kinematic_car
DATASET_DIR=$PWD/dataset ./start.sh

# 显式指定本目录
./start.sh --scene $PWD/dataset --robot go2
```

运行 ONS2 导航（需要 ONS2 仓库和已构建的 `isaac_nav_ros2` 镜像）：

```bash
DATASET_DIR=$PWD/dataset ./start.sh --robot go2 --runtime ros2 --stack ons2 \
  --map office_0923_sim --ons2-root /path/to/Odin-Nav-Stack2
```

说明：

- 未设置变量时，`start.sh` 默认使用 `<repo>/dataset`，并始终把它传给 `docker compose`，
  由 compose 以只读方式挂载到 `/dataset`。
- 场景配置必须使用容器内路径引用文件，例如
  `scene.environment.usdz_path: "/dataset/my_scene.usdz"`。可通过 `SIM_CONFIG=...` 选择配置。
- 如果目录为空或不存在，`start.sh` 只给出警告；Isaac Sim 会在没有扫描场景的情况下运行。

## 获取演示数据

从自己的 Q9000 扫描结果生成 USDZ 场景、PGM/YAML 地图和 ONS2 航点的完整流程见
[docs/Use the ONS2 and simulator manual.zh-CN.md](../docs/Use%20the%20ONS2%20and%20simulator%20manual.zh-CN.md)。
演示数据只是把已经准备好的 USDZ 放到这里，让首次运行不需要扫描或转换。

请只使用自己有权使用的扫描数据，并遵守场地所有方和数据提供方的授权要求。
