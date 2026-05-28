"""
Go2FlatTerrainPolicy — Unitree Go2 四足机器人策略控制器
使用 Go2 USD 模型 + Spot 预训练 RL 策略权重 (实验性)

注意: Spot 策略是为 Spot 运动学训练的, 用在 Go2 上只是概念验证,
实际行走效果可能不佳. 正式使用需要训练 Go2 专用策略.
"""

import os
from pathlib import Path
from typing import Optional

import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.storage.native import get_assets_root_path
from omni.physx import get_physx_simulation_interface

_ASSETS_DIR = Path(__file__).parent / "assets"

# Go2 配色方案 (link 名 → sRGB 线性颜色)
_GO2_COLORS = {
    "base":       (0.02, 0.02, 0.02),   # 机身 — 黑
    "base_black": (0.01, 0.01, 0.01),   # 机身黑色部件
    "base_white": (0.85, 0.85, 0.85),   # 机身白色部件
    "Head_upper": (0.02, 0.02, 0.02),   # 头部上 — 黑
    "Head_lower": (0.02, 0.02, 0.02),   # 头部下 — 黑
    "hip":        (0.10, 0.10, 0.10),   # 髋关节 — 深灰
    "thigh":      (0.05, 0.05, 0.05),   # 大腿 — 近黑
    "calf":       (0.08, 0.08, 0.08),   # 小腿 — 深灰
    "foot":       (0.60, 0.60, 0.60),   # 脚 — 浅灰 (橡胶)
}


def _apply_go2_materials(prim_path: str):
    """给 Go2 各 link 的 visual mesh 绑定 UsdPreviewSurface 材质.

    URDF 转换后 visuals 是 USD instanceable, 材质不继承到 instance 内部.
    需要先 SetInstanceable(False) 解除实例化, 再绑定到真实 Mesh prim.
    """
    import omni.usd
    from pxr import UsdShade, Sdf, Gf, UsdGeom, Usd

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(prim_path)
    if not robot_prim.IsValid():
        return

    looks_path = f"{prim_path}/Looks_go2"
    stage.DefinePrim(looks_path, "Scope")
    bound = 0

    for child in robot_prim.GetChildren():
        child_name = child.GetName()
        # 按最长 key 优先匹配 (base_black > base)
        color = None
        matched_key = ""
        for key, c in _GO2_COLORS.items():
            if key in child_name and len(key) > len(matched_key):
                color = c
                matched_key = key
        if color is None:
            continue

        visuals_path = f"{child.GetPath()}/visuals"
        visuals_prim = stage.GetPrimAtPath(visuals_path)
        if not visuals_prim.IsValid():
            continue

        # 解除 instanceable 才能访问子 mesh 并绑定材质
        if visuals_prim.IsInstanceable():
            visuals_prim.SetInstanceable(False)

        # 创建材质
        mat_path = f"{looks_path}/mat_{child_name}"
        mat_prim = stage.DefinePrim(mat_path, "Material")
        mat = UsdShade.Material(mat_prim)
        shader_prim = stage.DefinePrim(f"{mat_path}/Shader", "Shader")
        shader = UsdShade.Shader(shader_prim)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.1)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # 绑定到所有真实 Mesh prim
        for desc in Usd.PrimRange(visuals_prim):
            if desc.GetTypeName() == "Mesh":
                UsdShade.MaterialBindingAPI.Apply(desc)
                UsdShade.MaterialBindingAPI(desc).Bind(mat)
                bound += 1

    if bound > 0:
        print(f"  Go2 材质: 绑定 {bound} 个 mesh")


# Go2 默认关节角度 (来自 unitree_rl_gym/legged_gym/envs/go2/go2_config.py)
_GO2_DEFAULT_ANGLES = {
    "FL_hip_joint": 0.1,
    "FR_hip_joint": -0.1,
    "RL_hip_joint": 0.1,
    "RR_hip_joint": -0.1,
    "FL_thigh_joint": 0.8,
    "FR_thigh_joint": 0.8,
    "RL_thigh_joint": 1.0,
    "RR_thigh_joint": 1.0,
    "FL_calf_joint": -1.5,
    "FR_calf_joint": -1.5,
    "RL_calf_joint": -1.5,
    "RR_calf_joint": -1.5,
}


class Go2FlatTerrainPolicy(PolicyController):
    """Unitree Go2 quadruped using Spot's pre-trained flat terrain policy"""

    # 只保留这些 link 的碰撞体 (足部+机身), 其余删除以减少 PhysX 接触点
    _KEEP_COLLISION_LINKS = {
        "base", "FL_foot", "FR_foot", "RL_foot", "RR_foot",
        "FL_calf", "FR_calf", "RL_calf", "RR_calf",
    }

    # 目标总顶点数 (Spot ≈ 50K)
    _TARGET_TOTAL_VERTS = 50000

    @staticmethod
    def _decimate_mesh(mesh_prim, target_faces):
        """Vertex-clustering 减面: 快速、无外部依赖、保持拓扑.

        算法: 将顶点空间划分为 3D 网格, 每个格子内的顶点合并为重心,
        面索引重映射, 退化面 (2+顶点合并到同一格子) 自动删除.
        """
        from pxr import UsdGeom, Vt, Gf

        mesh = UsdGeom.Mesh(mesh_prim)
        pts_attr = mesh.GetPointsAttr()
        fvi_attr = mesh.GetFaceVertexIndicesAttr()
        fvc_attr = mesh.GetFaceVertexCountsAttr()

        pts = pts_attr.Get()
        fvi = fvi_attr.Get()
        fvc = fvc_attr.Get()
        if pts is None or fvi is None or len(pts) < 10:
            return 0, 0

        pts_np = np.array(pts, dtype=np.float32)
        fvi_np = np.array(fvi, dtype=np.int32)
        fvc_np = np.array(fvc, dtype=np.int32) if fvc else np.full(len(fvi) // 3, 3, dtype=np.int32)

        n_orig = len(pts_np)
        n_faces_orig = len(fvc_np)

        if n_faces_orig <= target_faces:
            return n_orig, n_orig  # 已经够小

        # 计算网格分辨率: 让顶点合并到 ~target 个格子
        bbox_min = pts_np.min(axis=0)
        bbox_max = pts_np.max(axis=0)
        bbox_size = bbox_max - bbox_min + 1e-8
        # 根据面数比例估算需要的格子数
        ratio = max(target_faces / n_faces_orig, 0.01)
        # 格子边长: 越小保留越多细节
        cell_size = (bbox_size.prod() / (n_orig * ratio)) ** (1.0 / 3.0)
        cell_size = max(cell_size, 1e-6)

        # 量化顶点到格子索引
        grid_idx = ((pts_np - bbox_min) / cell_size).astype(np.int32)
        # 编码为唯一 key
        gx, gy, gz = grid_idx[:, 0], grid_idx[:, 1], grid_idx[:, 2]
        max_g = max(gx.max(), gy.max(), gz.max()) + 1
        keys = gx.astype(np.int64) * max_g * max_g + gy.astype(np.int64) * max_g + gz.astype(np.int64)

        # 唯一格子 → 新顶点索引
        unique_keys, inverse = np.unique(keys, return_inverse=True)
        n_new = len(unique_keys)

        # 每个新顶点 = 同格子内顶点的重心
        new_pts = np.zeros((n_new, 3), dtype=np.float64)
        counts = np.zeros(n_new, dtype=np.float64)
        np.add.at(new_pts, inverse, pts_np.astype(np.float64))
        np.add.at(counts, inverse, 1.0)
        new_pts /= counts[:, None]

        # 重映射面索引, 过滤退化面 (向量化)
        new_fvi = inverse[fvi_np]

        # 快速路径: 全部是三角面 (Go2 大部分 mesh 如此)
        if np.all(fvc_np == 3):
            tri = new_fvi.reshape(-1, 3)
            # 非退化: 三个顶点互不相同
            valid = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])
            tri = tri[valid]
            new_fvi_arr = tri.flatten()
            new_fvc_arr = np.full(len(tri), 3, dtype=np.int32)
        else:
            # 通用路径: 混合面
            new_fvi_list = []
            new_fvc_list = []
            idx = 0
            for fc in fvc_np:
                face = new_fvi[idx:idx + fc]
                idx += fc
                if len(set(face.tolist())) == fc:
                    new_fvi_list.extend(face.tolist())
                    new_fvc_list.append(int(fc))
            new_fvi_arr = np.array(new_fvi_list, dtype=np.int32)
            new_fvc_arr = np.array(new_fvc_list, dtype=np.int32)

        if len(new_fvi_arr) == 0:
            return n_orig, n_orig

        # 只保留被面引用的顶点 (去除孤立顶点)
        used = np.unique(new_fvi_arr)
        remap = np.full(n_new, -1, dtype=np.int32)
        remap[used] = np.arange(len(used), dtype=np.int32)
        final_pts = new_pts[used].astype(np.float32)
        final_fvi = remap[new_fvi_arr].tolist()
        final_fvc = new_fvc_arr.tolist()

        # 写回 USD
        pts_attr.Set(Vt.Vec3fArray([Gf.Vec3f(*p.tolist()) for p in final_pts]))
        fvi_attr.Set(Vt.IntArray(final_fvi))
        fvc_attr.Set(Vt.IntArray(final_fvc))

        # 清除法线 (让渲染器重新计算)
        normals_attr = mesh.GetNormalsAttr()
        if normals_attr and normals_attr.HasValue():
            normals_attr.Clear()

        return n_orig, len(final_pts)

    @staticmethod
    def _tune_physics(prim_path: str):
        """让 Go2 的物理/渲染开销对齐 Spot 级别.

        修改:
        1. solver iterations: pos 32→4, vel 16→1  (Spot: 4/0)
        2. 删除非必要碰撞体 (只保留足部+机身)
        3. 清理损坏的 primvar (避免 Hydra 每帧报错开销)
        4. 运行时减面: 167 万顶点 → ~5 万 (vertex clustering)
        """
        import time as _time
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Gf, Vt

        t0 = _time.perf_counter()
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(prim_path)
        if not robot_prim.IsValid():
            return

        col_removed = 0
        col_kept = 0
        primvar_cleaned = 0
        mat_stripped = 0
        visual_meshes = []   # (prim, n_verts) for decimation

        for prim in Usd.PrimRange(robot_prim):
            # ---- 1. 降低 articulation solver 迭代 ----
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                if not prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                    PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api = PhysxSchema.PhysxArticulationAPI(prim)
                old_pos = api.GetSolverPositionIterationCountAttr().Get()
                old_vel = api.GetSolverVelocityIterationCountAttr().Get()
                api.GetSolverPositionIterationCountAttr().Set(4)
                api.GetSolverVelocityIterationCountAttr().Set(1)
                print(f"  Go2 tune: solver pos {old_pos}→4, vel {old_vel}→1")

            # ---- 2. 精简碰撞体: 只保留足部+机身 ----
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                link_name = prim.GetName()
                parent = prim.GetParent()
                if parent and parent.GetName() != "":
                    link_name = parent.GetName()
                    gp = parent.GetParent()
                    if gp and gp.GetName() not in ("", prim_path.split("/")[-1]):
                        link_name = gp.GetName()

                if link_name in Go2FlatTerrainPolicy._KEEP_COLLISION_LINKS:
                    col_kept += 1
                else:
                    prim.RemoveAPI(UsdPhysics.CollisionAPI)
                    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
                    col_removed += 1

            # ---- 3. 清理 primvar + 收集 visual mesh + 去材质 ----
            if prim.GetTypeName() == "Mesh":
                # 清理所有非必要 primvar (st, normals 等)
                to_remove = []
                for prop in prim.GetProperties():
                    name = prop.GetName()
                    if name.startswith("primvars:st"):
                        to_remove.append(name)
                for name in to_remove:
                    prim.RemoveProperty(name)
                    primvar_cleaned += 1

                # 收集 visual mesh
                path_str = str(prim.GetPath())
                if "visuals" in path_str or "visual" in path_str:
                    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                    if pts and len(pts) > 100:
                        visual_meshes.append((prim, len(pts)))

            # ---- 去除材质绑定 (消除 OmniPBR shader 开销) ----
            # RemoveProperty 无法覆盖引用层, 必须用 UnbindAllBindings
            if prim.HasRelationship("material:binding"):
                try:
                    UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
                    mat_stripped += 1
                except Exception:
                    pass

        print(f"  Go2 tune: 碰撞 kept={col_kept}, removed={col_removed}, "
              f"primvar清理={primvar_cleaned}, 材质解绑={mat_stripped}")

        # 给所有 visual mesh 设置 displayColor (简单平面着色, 无 shader)
        _go2_colors = {
            "base": (0.15, 0.15, 0.15),
            "base_white": (0.85, 0.85, 0.85),
            "base_black": (0.1, 0.1, 0.1),
        }
        for mesh_prim, _ in visual_meshes:
            mesh = UsdGeom.Mesh(mesh_prim)
            parent_name = mesh_prim.GetParent().GetName() if mesh_prim.GetParent() else ""
            color = _go2_colors.get(parent_name, (0.3, 0.3, 0.3))
            mesh.GetDisplayColorAttr().Set(
                Vt.Vec3fArray([Gf.Vec3f(*color)])
            )

        # 删除 Looks (Material/Shader prims) — 即使解绑了, 留着也占内存
        looks_path = prim_path + "/Looks"
        looks_prim = stage.GetPrimAtPath(looks_path)
        if looks_prim.IsValid():
            stage.RemovePrim(looks_path)
            print(f"  Go2 tune: 已删除 {looks_path}")

        # ---- 4. 运行时减面 (vertex clustering) ----
        total_orig = sum(n for _, n in visual_meshes)
        target_total = Go2FlatTerrainPolicy._TARGET_TOTAL_VERTS
        if total_orig > target_total * 1.5 and visual_meshes:
            total_new = 0
            for mesh_prim, n_verts in visual_meshes:
                share = n_verts / total_orig
                target_faces = max(50, int(target_total * share * 2))
                n_old, n_new = Go2FlatTerrainPolicy._decimate_mesh(
                    mesh_prim, target_faces)
                total_new += n_new
            dt = (_time.perf_counter() - t0) * 1000
            print(f"  Go2 tune: 减面 {total_orig:,} → {total_new:,} 顶点 "
                  f"({(1-total_new/total_orig)*100:.0f}%↓) [{dt:.0f}ms]")
        else:
            dt = (_time.perf_counter() - t0) * 1000
            print(f"  Go2 tune: 顶点={total_orig:,} (无需减面) [{dt:.0f}ms]")

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        assets_root_path = get_assets_root_path()
        if usd_path is None:
            usd_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"

        super().__init__(name, prim_path, root_path, usd_path, position, orientation)

        # ---- 降低物理迭代 (Nucleus 默认 pos=32/vel=16, Spot 仅 4/0) ----
        self._tune_physics(prim_path)

        # 加载 Spot 的策略权重 + 环境配置
        self.load_policy(
            assets_root_path + "/Isaac/Samples/Policies/Spot_Policies/spot_policy.pt",
            assets_root_path + "/Isaac/Samples/Policies/Spot_Policies/spot_env.yaml",
        )
        self._action_scale = 0.25  # Go2 action scale (from go2_config)
        self._previous_action = np.zeros(12)
        self._policy_counter = 0

    def initialize(self, physics_sim_view=None) -> None:
        """
        自定义初始化: 绕过 Spot env yaml 的关节名匹配,
        直接设置 Go2 的关节属性.
        """
        self.robot.initialize(physics_sim_view=physics_sim_view)
        self.robot.get_articulation_controller().set_effort_modes("force")
        get_physx_simulation_interface().flush_changes()
        self.robot.get_articulation_controller().switch_control_mode("position")

        n_dof = len(self.robot.dof_names)
        print(f"  Go2 DOF count: {n_dof}")
        print(f"  Go2 joint names: {self.robot.dof_names}")

        # Go2 PD 增益 (from go2_config: stiffness=20, damping=0.5)
        stiffness = [20.0] * n_dof
        damping = [0.5] * n_dof
        self.robot._articulation_view.set_gains(stiffness, damping)

        # Go2 电机限制
        max_effort = [23.5] * n_dof
        self.robot._articulation_view.set_max_efforts(max_effort)
        get_physx_simulation_interface().flush_changes()
        max_vel = [30.0] * n_dof
        self.robot._articulation_view.set_max_joint_velocities(max_vel)

        # 默认站立姿态 — 按关节名匹配
        self.default_pos = np.zeros(n_dof)
        self.default_vel = np.zeros(n_dof)
        matched = 0
        for i, dof_name in enumerate(self.robot.dof_names):
            for key, val in _GO2_DEFAULT_ANGLES.items():
                if key in dof_name:
                    self.default_pos[i] = val
                    matched += 1
                    break
        print(f"  Go2 matched {matched}/{n_dof} joints to default angles")

    def _compute_observation(self, command):
        """
        与 Spot 相同的 48 维观测向量:
        [lin_vel_b(3), ang_vel_b(3), gravity_b(3), command(3),
         joint_pos_err(12), joint_vel(12), prev_action(12)]
        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        obs = np.zeros(48)
        obs[:3] = lin_vel_b
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        obs[9:12] = command

        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()

        # 只取前 12 个关节 (忽略 Go2 可能有的额外固定关节)
        n = min(12, len(current_joint_pos))
        obs[12 : 12 + n] = current_joint_pos[:n] - self.default_pos[:n]
        obs[24 : 24 + n] = current_joint_vel[:n]
        obs[36 : 36 + n] = self._previous_action[:n]

        return obs

    def forward(self, dt, command):
        """
        计算并应用关节位置目标
        """
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = self._compute_action(obs)
            self._previous_action = self.action.copy()

        n = min(12, len(self.default_pos))
        target = self.default_pos.copy()
        target[:n] += self.action[:n] * self._action_scale
        action = ArticulationAction(joint_positions=target)
        self.robot.apply_action(action)

        self._policy_counter += 1
