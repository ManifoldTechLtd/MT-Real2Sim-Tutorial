"""
scene_builder.py
场景搭建工具 — USDZ 环境导入、光照、地面、碰撞修复

迁移自 isaac_arm_sim/shared/scene_utils.py, 更新 import 路径.
"""

import os
import numpy as np
import yaml
from pathlib import Path


def load_config(config_path: str) -> dict:
    """加载 YAML 配置文件"""
    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def get_assets_root_path() -> str:
    """获取 Isaac Sim Nucleus assets 根路径"""
    from isaacsim.core.utils.nucleus import get_assets_root_path as _get_root
    return _get_root()


def create_lighting(world, cfg: dict = None):
    """
    添加场景光源: 一个 Dome Light (环境光) + 一个 Distant Light (主光)

    cfg 可选参数 (均有默认值):
      dome_intensity:    Dome Light 强度 (默认 500, 室内场景建议 1500~3000)
      distant_intensity: Distant Light 强度 (默认 3000, 室内场景建议 500~1000)
      distant_angle:     Distant Light 角直径 (默认 0.53)
      distant_rotation:  Distant Light 旋转角 [rx, ry, rz] (默认 [-45, 30, 0])
    """
    from pxr import UsdLux, Gf

    if cfg is None:
        cfg = {}

    stage = world.stage

    # Dome Light — 环境光，提供均匀基础照明
    dome_intensity = cfg.get("dome_intensity", 500.0)
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(dome_intensity)

    # Distant Light — 平行光，模拟太阳光，提供方向性照明和阴影
    distant_intensity = cfg.get("distant_intensity", 3000.0)
    distant_light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant_light.CreateIntensityAttr(distant_intensity)
    distant_light.CreateAngleAttr(cfg.get("distant_angle", 0.53))
    # 设置光源方向 (从上方斜射)
    rot = cfg.get("distant_rotation", [-45.0, 30.0, 0.0])
    from pxr import UsdGeom
    xform = UsdGeom.Xformable(distant_light.GetPrim())
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rot))

    print(f"  光照: Dome={dome_intensity}, Distant={distant_intensity}, rot={rot}")


def create_ground_plane(world, z_offset: float = 0.0):
    """添加地面"""
    world.scene.add_default_ground_plane(z_position=z_offset)


def create_environment(world, cfg: dict, physics_cfg: dict = None):
    """
    导入 USDZ 环境场景作为 USD Reference
    并为 mesh 添加碰撞体 (triangle mesh collision)

    physics_cfg 可选参数 (scene.physics):
      collision_approximation: "meshSimplification" | "sdf" | "none" | "convexDecomposition"
      sdf_resolution:          SDF 体素分辨率 (默认 256)
      sdf_subgrid_resolution:  SDF 子网格分辨率 (默认 6)
      sdf_bits_per_subgrid_pixel: 子网格像素位深 (默认 2)
      contact_offset:          碰撞检测提前量 (m, 默认 PhysX 内部值)
      rest_offset:             接触静息偏移 (m, 默认 PhysX 内部值)
    """
    from pxr import Usd, UsdGeom, UsdPhysics, Gf

    if physics_cfg is None:
        physics_cfg = {}

    usdz_path = os.path.expanduser(cfg["usdz_path"])
    if not os.path.isfile(usdz_path):
        print(f"[scene_builder] WARN: USDZ 文件不存在, 跳过环境加载并回退到默认地面: {usdz_path}")
        return False

    stage = world.stage

    # 创建 /World/Environment Xform 作为容器
    env_prim_path = "/World/Environment"
    env_xform = UsdGeom.Xform.Define(stage, env_prim_path)

    # 添加 USDZ 作为 reference (只读引用)
    env_prim = stage.GetPrimAtPath(env_prim_path)
    env_prim.GetReferences().AddReference(usdz_path)

    # 设置位置
    pos = cfg.get("position", [0.0, 0.0, 0.0])
    env_xform.AddTranslateOp().Set(Gf.Vec3d(*pos))

    # 设置旋转 (欧拉角 → XYZ)
    orient = cfg.get("orientation", [0.0, 0.0, 0.0])
    if any(v != 0.0 for v in orient):
        env_xform.AddRotateXYZOp().Set(Gf.Vec3f(*orient))

    # 设置缩放
    scale = cfg.get("scale", [1.0, 1.0, 1.0])
    if any(v != 1.0 for v in scale):
        env_xform.AddScaleOp().Set(Gf.Vec3d(*scale))

    # ---- 物理碰撞参数 ----
    col_approx = physics_cfg.get("collision_approximation", "meshSimplification")
    contact_offset = physics_cfg.get("contact_offset", None)
    rest_offset = physics_cfg.get("rest_offset", None)
    use_sdf = (col_approx == "sdf")

    # 修复 USDZ prim 属性：可见性 + mesh 碰撞 / 渲染属性
    mesh_count = 0
    for prim in Usd.PrimRange(env_prim):
        img = UsdGeom.Imageable(prim)
        # 确保所有 prim 对渲染管线可见（2DGS 导出的 USDZ 可能标记 mesh 为 invisible）
        if img.ComputeVisibility() == "invisible":
            img.MakeVisible()

        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            # 设置双面渲染，使深度 pass 在 mesh 内侧也能命中
            mesh.CreateDoubleSidedAttr(True)
            # 关闭细分（2DGS mesh 不应使用 catmullClark 细分）
            mesh.GetSubdivisionSchemeAttr().Set("none")
            # 添加碰撞体
            if cfg.get("add_collision", True):
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(prim)
                if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                    mesh_col.CreateApproximationAttr(col_approx)

                # PhysX 扩展碰撞属性 (contact offset / rest offset)
                _apply_physx_collision(prim, contact_offset, rest_offset)

                # SDF 碰撞扩展属性
                if use_sdf:
                    _apply_physx_sdf(prim, physics_cfg)

            mesh_count += 1

    col_desc = col_approx
    if contact_offset is not None:
        col_desc += f", contactOffset={contact_offset}"
    if rest_offset is not None:
        col_desc += f", restOffset={rest_offset}"
    print(f"  环境修复: {mesh_count} 个 mesh (visible + doubleSided + subdivision=none + {col_desc})")

    print(f"  环境场景已加载: {usdz_path}")
    return env_prim


def _apply_physx_collision(prim, contact_offset=None, rest_offset=None):
    """
    应用 PhysxCollisionAPI 设置 contactOffset / restOffset.
    增大 contactOffset 可减少高速穿模; 正 restOffset 让物体在接触面略上方静止, 减少抖动.
    """
    if contact_offset is None and rest_offset is None:
        return

    try:
        from pxr import PhysxSchema
    except ImportError:
        print("  [WARN] PhysxSchema 不可用, 跳过 contact/rest offset 设置")
        return

    if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        PhysxSchema.PhysxCollisionAPI.Apply(prim)
    physx_col = PhysxSchema.PhysxCollisionAPI(prim)

    if contact_offset is not None:
        physx_col.CreateContactOffsetAttr(float(contact_offset))
    if rest_offset is not None:
        physx_col.CreateRestOffsetAttr(float(rest_offset))


def _apply_physx_sdf(prim, physics_cfg: dict):
    """
    应用 PhysxSDFMeshCollisionAPI 设置 SDF 碰撞参数.
    SDF 对重建 mesh 的薄面片和小漏洞鲁棒性远优于 meshSimplification.
    """
    try:
        from pxr import PhysxSchema
    except ImportError:
        print("  [WARN] PhysxSchema 不可用, 跳过 SDF 碰撞设置")
        return

    if not prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
        PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
    sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI(prim)

    sdf_resolution = physics_cfg.get("sdf_resolution", 256)
    sdf_api.CreateSdfResolutionAttr(int(sdf_resolution))

    sdf_subgrid = physics_cfg.get("sdf_subgrid_resolution", 6)
    sdf_api.CreateSdfSubgridResolutionAttr(int(sdf_subgrid))

    sdf_bits = physics_cfg.get("sdf_bits_per_subgrid_pixel", 2)
    sdf_api.CreateSdfBitsPerSubgridPixelAttr(
        "BitsPerPixel%d" % int(sdf_bits) if int(sdf_bits) in (1, 2, 4) else "BitsPerPixel2"
    )
