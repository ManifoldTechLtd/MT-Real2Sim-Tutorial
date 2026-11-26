"""Go2 MoE-CTS + velocity PI adapter for the navigation simulator."""
from typing import Optional
import numpy as np
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.prims import define_prim
from omni.physx import get_physx_scene_query_interface
from pxr import PhysxSchema, Usd, UsdPhysics
from r2s2r.core.go2_moe_runtime import (
    Go2MoeRuntime, JOINT_ORDER, DEFAULT_POS, TORQUE_LIMIT, POLICY_DT, PHYSICS_DT, ASSETS,
)

class Go2MoeCTSPolicy:
    def __init__(self, prim_path: str, name="go2", position: Optional[np.ndarray]=None,
                 orientation: Optional[np.ndarray]=None, usd_path=None, pi_enabled=True):
        if usd_path is None:
            usd_path = str(ASSETS/"usd/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd")
        prim = define_prim(prim_path, "Xform")
        prim.GetReferences().AddReference(usd_path)
        SingleXFormPrim(prim_path, position=position, orientation=orientation)
        root_path = None
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                root_path = child.GetPath().pathString
                art = PhysxSchema.PhysxArticulationAPI.Apply(child)
                art.CreateSolverPositionIterationCountAttr(8)
                art.CreateSolverVelocityIterationCountAttr(4)
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                body = PhysxSchema.PhysxRigidBodyAPI.Apply(child)
                body.CreateLinearDampingAttr(0.)
                body.CreateAngularDampingAttr(0.)
                body.CreateMaxLinearVelocityAttr(1000.)
                body.CreateMaxAngularVelocityAttr(1000.)
                body.CreateMaxDepenetrationVelocityAttr(1.)
        if root_path is None:
            raise RuntimeError(f"No articulation root in {usd_path}")
        self.robot = SingleArticulation(prim_path=root_path, name=name)
        self.runtime = Go2MoeRuntime(pi_enabled=pi_enabled)
        self._robot_prefix = prim_path.rstrip("/")+"/"
        self._scene_query = get_physx_scene_query_interface()
        self.default_pos = None
        self._step = 0

    def initialize(self, physics_sim_view=None):
        self.robot.initialize(physics_sim_view=physics_sim_view)
        names = list(self.robot.dof_names)
        self._perm = np.array([names.index(j) for j in JOINT_ORDER])
        if len(names) != 12:
            raise ValueError(f"Expected 12 Go2 joints, found {names}")
        self.robot.get_articulation_controller().switch_control_mode("effort")
        view = self.robot._articulation_view
        # Disable implicit drives; calculate the verified 20/0.5 PD every physics step.
        view.set_gains(np.zeros((1,12),np.float32), np.zeros((1,12),np.float32))
        limits = np.zeros(12,np.float32)
        limits[self._perm] = TORQUE_LIMIT
        view.set_max_efforts(limits.reshape(1,-1))
        materials = view._physics_view.get_material_properties()
        materials[:] = (1.,1.,0.)
        view._physics_view.set_material_properties(materials, np.arange(view._physics_view.count,dtype=np.int32))
        self.default_pos = np.zeros(12,np.float32)
        self.default_pos[self._perm] = DEFAULT_POS
        position,orientation = self.robot.get_world_pose()
        position = np.asarray(position).copy()
        distance = self._ground_distance(position)
        if distance is not None:
            position[2] = position[2]-distance+.4
        self.robot.set_world_pose(position,orientation)
        self.robot.set_linear_velocity(np.zeros(3))
        self.robot.set_angular_velocity(np.zeros(3))
        self.robot.set_joint_positions(self.default_pos)
        self.robot.set_joint_velocities(np.zeros(12))
        self.runtime.reset()
        self._step = 0
        print(f"  Go2 MoE-CTS: 45 observations, internal history=10, PI={self.runtime.pi_enabled}; dof={names}; z={position[2]:.3f}")

    def forward(self, dt, command):
        if not np.isfinite(dt) or abs(dt-PHYSICS_DT)>1e-6:
            raise ValueError(f"Go2 MoE-CTS requires physics_dt={PHYSICS_DT}, got {dt}")
        q = self.robot.get_joint_positions()[self._perm]
        dq = self.robot.get_joint_velocities()[self._perm]
        decimation = round(POLICY_DT/dt)
        if self._step and self._step % decimation == 0:
            _,orientation = self.robot.get_world_pose()
            self.runtime.infer(q,dq,orientation,self.robot.get_linear_velocity(),
                               self.robot.get_angular_velocity(),command,self._step*dt)
        efforts = np.empty(12,np.float32)
        efforts[self._perm] = self.runtime.effort(q,dq)
        self.robot.set_joint_efforts(efforts)
        self._step += 1

    def _ground_distance(self, position):
        origin = tuple(float(v) for v in position)
        hit = self._scene_query.raycast_closest(origin,(0.,0.,-1.),20.,bothSides=True)
        if not hit["hit"]:
            return None
        if not hit["rigidBody"].startswith(self._robot_prefix):
            return hit["distance"]
        nearest = []
        def keep_external(h):
            if not h.rigid_body.startswith(self._robot_prefix):
                nearest.append(h.distance)
            return True
        self._scene_query.raycast_all(origin,(0.,0.,-1.),20.,keep_external,bothSides=True)
        return min(nearest) if nearest else None
