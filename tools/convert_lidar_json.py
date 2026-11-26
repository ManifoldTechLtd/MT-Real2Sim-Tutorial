#!/usr/bin/env python3
"""将 LiDAR JSON profile 转换为 OmniLidar .usd 文件 (Isaac Sim 5.x).

用法 (在 Isaac Sim Python 环境下运行):
    python tools/convert_lidar_json.py <json_path> [json_path2 ...]

示例:
    python tools/convert_lidar_json.py quadruped_demo/lidar_configs/ML30S_dense.json

生成的 .usd 文件输出到与 json/ 平行的 usd/ 目录, 例如:
    lidar_configs/json/ML30S_dense.json -> lidar_configs/usd/ML30S_dense.usd

背景:
  Isaac Sim 5.x 的 OmniLidar prim 不再使用 JSON + sensorModelConfig,
  而是将配置存储在 USD 属性 (omni:sensor:Core:*) 中.
  本脚本封装了官方 convert_lidar_json_to_usda.py 工具.
"""
import sys
import os

# 官方转换工具路径
_ISAACSIM_ROOT = os.environ.get("ISAACSIM_PATH", os.path.expanduser("~/isaacsim"))
_CONVERT_TOOL = os.path.join(
    _ISAACSIM_ROOT, "tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py"
)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    json_files = [os.path.abspath(f) for f in sys.argv[1:]]
    for f in json_files:
        if not os.path.isfile(f):
            print(f"ERROR: 文件不存在: {f}")
            sys.exit(1)

    # 启动 Isaac Sim (必须在 import 其他 omni 模块之前)
    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": True})

    # 动态加载官方转换工具
    import importlib.util
    spec = importlib.util.spec_from_file_location("_convert_tool", _CONVERT_TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    model_config = {
        "marketName": "Generic",
        "modelName": "LidarCore",
        "modelVendor": "NVIDIA",
        "modelVersion": "0.0.0",
    }

    for json_path in json_files:
        # 输出到与 json/ 平行的 usd/ 目录，匹配 sensor_suite.py 的查找逻辑
        json_dir = os.path.dirname(json_path)
        json_stem = os.path.splitext(os.path.basename(json_path))[0]
        usd_dir = os.path.join(os.path.dirname(json_dir), "usd")
        os.makedirs(usd_dir, exist_ok=True)
        usd_path = os.path.join(usd_dir, f"{json_stem}.usd")
        print(f"Converting: {json_path}")
        print(f"       --> {usd_path}")
        mod.convert_json_to_usd(json_path, usd_path, model_config)
        print(f"  OK: {os.path.getsize(usd_path)} bytes")

    kit.close()
    print(f"\nDone. Converted {len(json_files)} file(s).")


if __name__ == "__main__":
    main()
