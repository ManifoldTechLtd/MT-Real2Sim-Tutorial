"""
robot_factory.py
多机器人工厂 — 支持 Spot / Go2 (RL policy 四足) 与 kinematic_car (运动学差速车).

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
        # Go2-specific MoE-CTS with velocity PI; 50 Hz inference.
        "class": "r2s2r.core.go2_moe_policy.Go2MoeCTSPolicy",
        "body_link": "base",
        "physics_dt": 0.005,  # Isaac Lab sim.dt; 50 Hz policy, explicit PD at 200 Hz
    },
    "kinematic_car": {
        # 运动学差速车: 旁路物理引擎, 适合验证导航算法 (路径规划/控制)
        "class": "r2s2r.core.kinematic_vehicle.KinematicVehicle",
        "body_link": "base_link",
    },
}


def physics_dt_for(robot_type: str) -> float:
    return _ROBOT_REGISTRY.get(robot_type, {}).get("physics_dt", 1.0 / 500.0)


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

    # 仅 kinematic_car 支持额外 kwargs (size / base_link_z_above_ground / 等).
    # SpotFlatTerrainPolicy / Go2FlatTerrainPolicy 的构造签名固定, 不接受额外参数.
    extra_kwargs = {}
    if robot_type == "kinematic_car":
        extra_kwargs = dict(cfg.get("kinematic_car", {}) or {})
        # 不属于 KinematicVehicle ctor 的键 (给上层管线用, 如 run_sim.py 自身回波过滤),
        # 在此剥离避免 TypeError: unexpected keyword argument.
        for _k in ("lidar_self_filter",):
            extra_kwargs.pop(_k, None)

    robot = RobotClass(
        prim_path=prim_path,
        name=robot_type,
        position=position,
        **extra_kwargs,
    )

    body_link = info["body_link"]
    body_prim_path = f"{prim_path}/{body_link}"

    print(f"  机器人: {robot_type} @ {prim_path} (body: {body_prim_path})")
    return robot, body_prim_path


def _build_common_scene(world, config: dict):
    """搭建公共场景元素: 环境、光源、地面"""
    scene_cfg = config["scene"]

    # 环境场景 (USDZ)
    env_cfg = scene_cfg.get("environment", {})
    if env_cfg.get("enabled", False):
        physics_cfg = scene_cfg.get("physics", {})
        create_environment(world, env_cfg, physics_cfg=physics_cfg)

    # 光源
    create_lighting(world, config.get("lighting"))

    # 地面
    if scene_cfg.get("use_default_ground", True):
        create_ground_plane(world, z_offset=scene_cfg.get("ground_z_offset", 0.0))


def build_quadruped_scene(world, config: dict):
    """
    一键搭建四足机器人导航场景.

    Returns:
        (robot_controller, sensor_suite_or_None, body_prim_path)
        robot_controller: PolicyController 子类 (有 .forward(dt, cmd), .initialize() 等方法)
        body_prim_path: 机器人 body prim 路径 (body_link 由 _ROBOT_REGISTRY 统一维护,
                        调用方不要再自建映射表 — 两处映射不同步会导致传感器/点云失效)
    """
    _build_common_scene(world, config)

    # 四足机器人
    quad_cfg = dict(config["quadruped"])  # 浅拷贝, 避免污染原 config
    # 顶层 kinematic_car 段 (yaml 习惯把几何参数和 quadruped 平铺) 注入到 quad_cfg, 让 factory 透传到 ctor
    if "kinematic_car" in config and "kinematic_car" not in quad_cfg:
        quad_cfg["kinematic_car"] = config["kinematic_car"]
    robot, body_prim_path = create_quadruped(quad_cfg)

    # 传感器套件 (可选)
    ss_cfg = config.get("sensor_suite") or {}
    calibration_dir = (ss_cfg.get("calibration_dir_by_robot") or {}).get(
        quad_cfg.get("robot_type", "spot"), ss_cfg.get("calibration_dir"))
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
        camera_render_scale = float(
            (config.get("sensor_suite") or {}).get("camera_render_scale", 1.0))
        clip_kwargs = {}
        clip_near = (config.get("sensor_suite") or {}).get("camera_clip_near")
        clip_far = (config.get("sensor_suite") or {}).get("camera_clip_far")
        if clip_near is not None:
            clip_kwargs['camera_clip_near'] = float(clip_near)
        if clip_far is not None:
            clip_kwargs['camera_clip_far'] = float(clip_far)
        suite = SensorSuite(calibration_dir, parent_prim, sim_z_offset=sim_z_offset,
                            camera_render_scale=camera_render_scale, **clip_kwargs)
        suite.build(world)

    return robot, suite, body_prim_path
