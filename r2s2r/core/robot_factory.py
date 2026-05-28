"""
robot_factory.py
多机器人工厂 — 支持 Spot (快速验证) 和 Go2 (最终目标)

迁移自 isaac_arm_sim/quadruped_demo/builder.py,
移除 ANYmal/H1, 更新 import 路径.
"""

import os
import numpy as np
from pathlib import Path

from r2s2r.core.scene_builder import (
    create_environment, create_lighting, create_ground_plane,
)

_PROJECT_DIR = Path(__file__).resolve().parent.parent  # isaacsim/


# ---- 机器人注册表 ----

_ROBOT_REGISTRY = {
    "spot": {
        "class": "isaacsim.robot.policy.examples.robots.SpotFlatTerrainPolicy",
        "body_link": "body",
    },
    "go2": {
        "class": "r2s2r.core.go2_policy.Go2FlatTerrainPolicy",
        "body_link": "base",
    },
}


def create_quadruped(cfg: dict):
    """
    根据 robot_type 创建四足机器人 (PolicyController 子类).
    返回 (robot_controller, body_prim_path)
    """
    robot_type = cfg.get("robot_type", "spot")
    prim_path = cfg.get("prim_path", "/World/Quadruped")
    position = np.array(cfg.get("position", [0, 0, 0.8]))

    info = _ROBOT_REGISTRY.get(robot_type)
    if info is None:
        raise ValueError(
            f"未知 robot_type: '{robot_type}', 可选: {list(_ROBOT_REGISTRY.keys())}"
        )

    # 动态导入对应的类
    module_path, class_name = info["class"].rsplit(".", 1)
    import importlib
    mod = importlib.import_module(module_path)
    RobotClass = getattr(mod, class_name)

    robot = RobotClass(
        prim_path=prim_path,
        name=robot_type,
        position=position,
    )

    body_link = info["body_link"]
    body_prim_path = f"{prim_path}/{body_link}"

    print(f"  机器人: {robot_type} @ {prim_path} (body: {body_prim_path})")
    return robot, body_prim_path


def _build_common_scene(world, config: dict):
    """搭建公共场景元素: 环境、光源、地面"""
    scene_cfg = config["scene"]

    # 环境场景 (USDZ)
    env_loaded = False
    env_cfg = scene_cfg.get("environment", {})
    if env_cfg.get("enabled", False):
        physics_cfg = scene_cfg.get("physics", {})
        env_loaded = create_environment(world, env_cfg, physics_cfg=physics_cfg) is not False

    # 光源
    create_lighting(world, config.get("lighting"))

    # 地面 (USDZ 未加载时强制启用默认地面, 保证机器人有支撑)
    if scene_cfg.get("use_default_ground", True) or not env_loaded:
        create_ground_plane(world, z_offset=scene_cfg.get("ground_z_offset", 0.0))


def build_quadruped_scene(world, config: dict):
    """
    一键搭建四足机器人导航场景.

    Returns:
        (robot_controller, sensor_suite_or_None)
        robot_controller: PolicyController 子类 (有 .forward(dt, cmd), .initialize() 等方法)
    """
    _build_common_scene(world, config)

    # 四足机器人
    quad_cfg = config["quadruped"]
    robot, body_prim_path = create_quadruped(quad_cfg)

    # 传感器套件 (可选)
    calibration_dir = (config.get("sensor_suite") or {}).get("calibration_dir")
    suite = None
    if calibration_dir:
        calibration_dir = os.path.expanduser(calibration_dir)
        if not os.path.isabs(calibration_dir):
            # 相对于 configs/ 目录查找
            calibration_dir = os.path.join(str(_PROJECT_DIR / "configs"), calibration_dir)

        from r2s2r.core.sensor_suite import SensorSuite

        # 如果 config 中指定了 parent_prim_path 就使用, 否则用自动检测的 body_prim_path
        parent_prim = (config.get("sensor_suite") or {}).get(
            "parent_prim_path", body_prim_path
        )
        sim_z_offset = (config.get("sensor_suite") or {}).get("sim_z_offset", 0.0)
        suite = SensorSuite(calibration_dir, parent_prim, sim_z_offset=sim_z_offset)
        suite.build(world)

    return robot, suite
