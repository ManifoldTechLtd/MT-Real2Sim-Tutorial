# 自定义 LiDAR 配置指南

[English](lidar_guide.md)

本文档记录 Isaac Sim RTX LiDAR 自定义配置的完整流程与已知坑点。

---

## 1. 当前使用的 LiDAR

**MT-MLXS** — 基于 ZVISION ML-Xs 的定制版本。

| 参数 | 值 |
|------|-----|
| 类型 | 固态 (solidState) |
| 水平 FOV | ≈123° (-61.3° ~ 61.5°) |
| 垂直 FOV | ≈32° (-14.0° ~ 17.7°) |
| 每帧点数 | 27,000 (原版 108,000 的 1/4) |
| 扫描线 | 180 线 × 150 点/线 |
| 测距范围 | 0.5m ~ 250m |
| 精度 | 0.03m 分辨率, 0.05m 精度 |

**相比原版 ZVISION ML-Xs 的改动：**
- `fireTimeNs` 全部置零 — 取消发射时序，单帧同时返回所有点（无运动畸变模拟）
- 每线点数从 600 降到 150 — 降低点云传输到 RViz 的延迟

文件位置：
- JSON 配置: `lidar_configs/json/MT_MLXS.json`
- USD 文件: `lidar_configs/usd/MT_MLXS.usd`
- 内参引用: `r2s2r/configs/calibration/pinhole/intrinsics_lidar1.yaml`

---

## 2. 文件结构

```
lidar_configs/
├── json/
│   └── MT_MLXS.json          # LiDAR profile (Isaac Sim RTX JSON 格式)
└── usd/
    └── MT_MLXS.usd           # 转换后的 USD 文件 (运行时实际加载)
```

**重要**: Isaac Sim 5.x 运行时只读 USD，不读 JSON。JSON 是源文件，USD 是编译产物。

---

## 3. 新建/修改自定义 LiDAR 的流程

### 3.1 准备 JSON 配置

JSON 格式遵循 Isaac Sim RTX LiDAR profile 规范。可从以下来源获取模板：

- ZVISION 官方: https://github.com/ZVISION-lidar/ZVISION-lidar.github.io
- Isaac Sim 内置配置目录: `${ISAACSIM_PATH}/tools/isaacsim.sensors.rtx/`

关键字段：
```json
{
  "name": "MT-MLXS",
  "class": "sensor",
  "type": "lidar",
  "profile": {
    "scanType": "solidState",
    "nearRangeM": 0.5,
    "farRangeM": 250,
    "numRaysPerLine": [150, 150, ...],       // 每线点数
    "emitterStates": [{
      "azimuthDeg": [...],                    // 水平角度列表
      "elevationDeg": [...],                  // 垂直角度列表
      "fireTimeNs": [0, 0, ...]              // 发射时间 (全零=单帧全量)
    }]
  }
}
```

### 3.2 JSON → USD 转换

**必须在 Isaac Sim Python 环境下运行**（容器内或本地安装均可）：

```bash
# 在 Isaac Sim 容器内:
python tools/convert_lidar_json.py lidar_configs/json/MT_MLXS.json

# 输出: lidar_configs/usd/MT_MLXS.usd
```

### 3.3 配置引用

编辑 `r2s2r/configs/calibration/pinhole/intrinsics_lidar1.yaml`：

```yaml
model: rtx_lidar
config_name: "lidar_configs/json/MT_MLXS.json"
```

`config_name` 以 `.json` 结尾时，`sensor_suite.py` 会自动查找对应的 `.usd` 文件。

查找逻辑：给定 `lidar_configs/json/MT_MLXS.json`，按以下顺序搜索 USD：
1. `lidar_configs/json/MT_MLXS.usd` (同目录)
2. `lidar_configs/usd/MT_MLXS.usd` (平行 usd/ 目录) ← 通常命中

---

## 4. 已知坑点

### 4.1 Isaac Sim 5.x 不再支持运行时读 JSON

Isaac Sim 4.x 可以通过 `sensorModelConfig` 属性直接指向 JSON 文件。
**Isaac Sim 5.x 改为 USD prim 属性 (`omni:sensor:Core:*`)**，必须先用转换工具生成 USD。

如果忘记转换，`sensor_suite.py` 会找不到 USD 文件并**静默回退到 `Example_Rotary`**（内置旋转式 LiDAR），表现为点云模式完全不同。

### 4.2 转换脚本依赖 Isaac Sim Python 环境

`tools/convert_lidar_json.py` 内部调用 `isaacsim.SimulationApp` 和 Isaac Sim 官方的 `convert_lidar_json_to_usda.py`。**不能用普通 Python 运行**。

官方工具路径：
```
${ISAACSIM_PATH}/tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py
```

如果 Isaac Sim 安装路径非默认 (`~/isaacsim`)，需设置环境变量：
```bash
export ISAACSIM_PATH=/path/to/isaacsim
```

### 4.3 USD 文件需要提交到版本控制

USD 是二进制文件，但因为转换依赖 Isaac Sim 环境，建议**将 USD 一同提交到 Git**，避免用户部署时还需额外运行转换。

### 4.4 JSON 路径解析

`sensor_suite.py` 中 `config_name` 是相对路径，解析时按以下目录顺序搜索：
1. `r2s2r/configs/` (calibration 的上级)
2. `r2s2r/` (项目模块根)
3. 工作区根目录 (`mt-real2sim-tutorial/`) ← 通常命中

如果路径找不到，检查 Isaac Sim 控制台中 `[LiDAR Debug]` 开头的日志。

### 4.5 修改 JSON 后必须重新转换 USD

JSON 和 USD 不会自动同步。每次修改 JSON 后都需要重新运行：
```bash
python tools/convert_lidar_json.py lidar_configs/json/MT_MLXS.json
```

---

## 5. 切换到 Isaac Sim 内置 LiDAR

如果不需要自定义配置，可以直接使用内置名称：

```yaml
# intrinsics_lidar1.yaml
model: rtx_lidar
config_name: "ZVISION_MLXS"       # 内置配置名，无需 JSON/USD
```

常用内置配置：
- `Example_Rotary` — 旋转式, 130K+ pts/frame
- `ZVISION_ML30S` — 固态, 51K pts, 153°×73°, 45m
- `ZVISION_MLXS` — 固态, 108K pts, 123°×32°, 250m
