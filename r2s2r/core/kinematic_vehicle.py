"""
kinematic_vehicle.py
运动学差速驱动车 — 旁路物理引擎, 直接积分 cmd_vel -> base pose, 用 raycast 跟地形.

设计原则:
  - 不用 RL 策略, 不用 articulation, 不模拟轮地摩擦.
  - 控制接口: forward(dt, cmd=[vx, vy, wz]),  履带车 vy 强制忽略.
  - 地形跟随: 每步向下 raycast 找命中 z, base_link.z = z_hit + base_link_z_above_ground.
  - 与 SpotFlatTerrainPolicy / Go2FlatTerrainPolicy 同 duck-type 接口
    (initialize / forward / .robot.{get_world_pose,set_world_pose,set_linear_velocity,set_angular_velocity,set_joint_positions}).
  - 与 SensorSuite 兼容: parent_prim 仍为 base_link (Xform), 传感器作为子 prim.
"""

import math
import numpy as np

from pxr import UsdGeom, Gf, Usd


def _yaw_from_quat_wxyz(w, x, y, z):
    """从 (w, x, y, z) 四元数提取 yaw (Z 轴旋转, 弧度).

    Hamilton 约定, scalar-first.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_wxyz_from_yaw(yaw):
    """yaw -> (w, x, y, z) 四元数."""
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


class _PoseHandle:
    """模仿 articulation.robot 接口的 pose 持有器.

    提供给上层 (如 run_sim.py 的 reset_robot) 与 SpotFlatTerrainPolicy.robot
    一致的方法签名: get_world_pose / set_world_pose / set_linear_velocity /
    set_angular_velocity / set_joint_positions / set_joint_velocities.

    velocity / joint 相关方法均为 noop (无物理 / 无关节).
    """

    def __init__(self, parent_vehicle):
        self._vehicle = parent_vehicle

    def get_world_pose(self):
        """返回 (pos_xyz, quat_wxyz), 与 Isaac Sim Articulation 一致.

        Returns:
            (np.ndarray shape=(3,), np.ndarray shape=(4,)) 或 (None, None)
        """
        prim = self._vehicle._base_prim
        if prim is None or not prim.IsValid():
            return None, None
        mat = UsdGeom.Imageable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        m = np.array(mat).T  # 4x4
        pos = np.array([m[0, 3], m[1, 3], m[2, 3]], dtype=np.float64)
        # 旋转矩阵 -> quat (w, x, y, z): Shepperd 4 分支法, 在任意旋转下数值
        # 稳定. 单分支 (tr > 0) 版本在 |yaw| > 120° 时静默退化为 identity,
        # 表现为 "机头朝后 TF 朝前", 见 run_sim.py mat_to_quat 注释.
        R = m[:3, :3]
        m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
        tr = m00 + m11 + m22
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return pos, np.array([qw, qx, qy, qz], dtype=np.float64)

    def set_world_pose(self, pos, quat_wxyz):
        self._vehicle._apply_pose(
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(quat_wxyz[0]), float(quat_wxyz[1]),
            float(quat_wxyz[2]), float(quat_wxyz[3]),
        )

    def set_linear_velocity(self, *_args, **_kwargs):
        # 运动学模式无物理速度, 仅清零内部积分状态.
        self._vehicle._reset_motion_state()

    def set_angular_velocity(self, *_args, **_kwargs):
        self._vehicle._reset_motion_state()

    def set_joint_positions(self, *_args, **_kwargs):
        return  # 无关节

    def set_joint_velocities(self, *_args, **_kwargs):
        return


class KinematicVehicle:
    """运动学差速驱动车控制器.

    生命周期与 SpotFlatTerrainPolicy 一致:
        v = KinematicVehicle(prim_path=..., name=..., position=..., ...)
        # build 阶段已在 __init__ 完成 (创建 prim).
        world.reset()
        v.initialize()             # 锁定 raycast / 设初始 z
        # 每物理步:
        v.forward(dt, cmd=[vx, vy, wz])

    与 PolicyController 子类的接口对齐:
        - 构造签名: (prim_path, name, position)
        - .initialize()
        - .forward(step_size, cmd)
        - .robot.{get_world_pose, set_world_pose, set_linear_velocity, ...}
        - .default_pos: 用于 reset_robot, 这里设 None (无关节).
    """

    BODY_LINK = "base_link"

    def __init__(
        self,
        prim_path: str = "/World/Quadruped",
        name: str = "kinematic_car",
        position=(0.0, 0.0, 0.2),
        *,
        size=(1.04, 0.77, 0.30),       # K02: length 1.04m, width 0.77m, height 0.30m
        base_link_z_above_ground: float = 0.20,
        raycast_max_dist: float = 5.0,           # 起点高 raycast_origin_above_base, 最多向下 max_dist
        raycast_origin_above_base: float = 1.5,
        # 4 履带组模型: 模拟前后左右各 1 条履带, 每条履带顶贴最高支撑点 (跨越凸起).
        # track_centers 是 4 履带组中心在车体系 (x, y); None 时用 0.45*size_x × 0.40*size_y.
        track_centers=None,
        track_subsample_count: int = 3,          # 每履带组沿前后方向子采样数 (履带长度方向)
        track_subsample_dx: float = 0.08,        # 子采样间距 (m), 履带前后接地点间隔
        max_climb_height: float = 0.60,          # 履带相对当前履带顶可攀爬的最大高度差.调小了上不去坡
                                                 # raycast 命中超过此高度视为障碍物 (汽车/墙/立柱),
                                                 # 从地形拟合中剔除 -> 由 overlap_box 水平碰撞处理.
        ground_normal_z_min: float = 0.3,        # hit.normal.z 大于此才认为是可走地面 (~73度)
        # 地形姿态拟合 (车头随坡面翘起)
        attitude_smoothing: float = 1.0,         # pitch/roll 一阶低通 (1.0=无滤波, 立即跟上目标)
                                                 # 平地拟合本身稳定, 默认无滤波避免上坡瞬态穿模
        max_tilt_deg: float = 35.0,              # pitch / roll 最大限幅, 防拟合崩坏
        # 履带物理: 碰撞盒底抬到 base_link.z + collision_box_bottom_above_base
        # 的高度, 此值以下全部视作"履带通过区" (与 max_climb_height 一致), 凹凸 / 石头 /
        # 沟坎 / 矮台阶都不阻挡; 仅高于此值的物体 (墙 / 栏杆 / 立柱) 算障碍.
        # None -> 用 max_climb_height 作默认, 让两个阈值保持一致.
        collision_box_bottom_above_base=None,
        collision_box_height: float = 0.5,       # 碰撞盒高度 (m), 用来撞栏杆 / 立柱
        collision_shrink: float = 0.92,
        enable_horizontal_collision: bool = True,
        debug_terrain: bool = False,             # 打印每帧地形拟合状态 (各履带 raycast z, 过滤情况)
        visual_color=(0.85, 0.55, 0.10),
        ignore_lateral: bool = True,
    ):
        self._prim_path = prim_path
        self._name = name
        self._base_link_path = f"{prim_path}/{self.BODY_LINK}"
        self._visual_path = f"{self._base_link_path}/chassis_visual"
        self._size = np.array(size, dtype=np.float64)
        self._base_link_z = float(base_link_z_above_ground)
        self._raycast_max_dist = float(raycast_max_dist)
        self._raycast_origin_above_base = float(raycast_origin_above_base)
        self._ground_normal_z_min = float(ground_normal_z_min)
        self._max_climb_height = float(max_climb_height)
        self._attitude_alpha = float(attitude_smoothing)
        self._max_tilt = math.radians(float(max_tilt_deg))
        # bottom 默认与 max_climb_height 联动 (履带通过区高度 = 攀爬阈值)
        if collision_box_bottom_above_base is None:
            self._collision_box_bottom_above_base = float(max_climb_height)
        else:
            self._collision_box_bottom_above_base = float(collision_box_bottom_above_base)
        self._collision_box_height = float(collision_box_height)
        self._collision_shrink = float(collision_shrink)
        self._enable_horizontal_collision = bool(enable_horizontal_collision)
        self._debug_terrain = bool(debug_terrain)
        self._debug_counter = 0
        self._debug_last_state = None  # 上一帧拟合状态 cache, 用于失败时打印细节
        # 启动期一次性打印 debug 参数, 方便确认 yaml -> ctor 传递成功
        print(f"[KINEMATIC INIT] debug_terrain={self._debug_terrain} "
              f"max_climb={self._max_climb_height:.2f} "
              f"max_tilt={math.degrees(self._max_tilt):.1f}°")
        self._visual_color = visual_color
        self._ignore_lateral = bool(ignore_lateral)
        # overlap_box 回调闭包用的 hit flag
        self._overlap_hit_flag = False

        # 4 履带组中心 (车体系). 默认 FL/FR/RL/RR 在车体 ±45% 长 × ±40% 宽处.
        if track_centers is None:
            L = float(self._size[0]) * 0.45
            W = float(self._size[1]) * 0.40
            track_centers = [
                (+L, +W),  # FL
                (+L, -W),  # FR
                (-L, +W),  # RL
                (-L, -W),  # RR
            ]
        self._track_centers = [(float(a), float(b)) for a, b in track_centers]
        self._track_n_sub = max(1, int(track_subsample_count))
        self._track_sub_dx = float(track_subsample_dx)

        # 内部 pose 状态 (积分用), 与 USD 上的 Xform 双向同步.
        init = np.asarray(position, dtype=np.float64)
        self._x = float(init[0])
        self._y = float(init[1])
        self._yaw = 0.0
        self._pitch = 0.0        # 车头抬起为正 (绕车体 y 轴)
        self._roll = 0.0         # 左侧抬起为正 (绕车体 x 轴)
        self._z = float(init[2]) if len(init) > 2 else self._base_link_z
        self._last_valid_z = self._z

        # USD prim 缓存
        self._stage = None
        self._base_prim = None
        self._base_xform = None
        self._translate_op = None
        self._orient_op = None
        self._scale_op = None

        # raycast 接口 (延迟到 initialize)
        self._raycast = None

        # 模仿 articulation 接口
        self.robot = _PoseHandle(self)
        self.default_pos = None

        # 建 prim
        self._build_usd()

    # ---------- USD 构建 ----------

    def _build_usd(self):
        """创建 /World/Quadruped Xform + /base_link Xform + chassis_visual Cube.

        chassis_visual 仅做可视化, 不加 collider (避免 raycast 撞自己).
        """
        import omni.usd
        self._stage = omni.usd.get_context().get_stage()

        # 顶层 Xform (如未存在)
        if not self._stage.GetPrimAtPath(self._prim_path).IsValid():
            UsdGeom.Xform.Define(self._stage, self._prim_path)

        # base_link Xform
        base_xform = UsdGeom.Xform.Define(self._stage, self._base_link_path)
        self._base_prim = base_xform.GetPrim()
        self._base_xform = base_xform

        # 清掉已有 xformOp (重复 build 时), 再添加我们要的三个 op.
        base_xform.ClearXformOpOrder()
        self._translate_op = base_xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        self._orient_op = base_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        self._scale_op = None  # 不缩放

        # 初始 pose
        self._translate_op.Set(Gf.Vec3d(self._x, self._y, self._z))
        self._orient_op.Set(Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)))

        # 车体可视化 Cube — chassis 在 base_link 几何中心, size = 1m^3, 通过 scale 缩到目标尺寸
        cube = UsdGeom.Cube.Define(self._stage, self._visual_path)
        cube.GetSizeAttr().Set(1.0)
        # 设置子 Xform 的 scale 把单位立方体变成车体尺寸
        cube_xform = UsdGeom.Xformable(cube.GetPrim())
        cube_xform.ClearXformOpOrder()
        scale_op = cube_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        scale_op.Set(Gf.Vec3d(float(self._size[0]), float(self._size[1]), float(self._size[2])))
        # 颜色
        try:
            cube.CreateDisplayColorAttr([Gf.Vec3f(*self._visual_color)])
        except Exception:
            pass

        print(f"[KINEMATIC] vehicle built @ {self._base_link_path} "
              f"size={tuple(self._size.tolist())} base_link_z_above_ground={self._base_link_z:.2f}m")

    # ---------- 生命周期 ----------

    def initialize(self):
        """锁定 PhysX raycast 接口; 用初始 (x,y) 探一次地形高度作为参考."""
        try:
            from omni.physx import get_physx_scene_query_interface
            self._raycast = get_physx_scene_query_interface()
        except Exception as e:
            print(f"[KINEMATIC WARN] PhysX raycast unavailable, fallback to fixed z: {e}")
            self._raycast = None

        # 探一次初始地形姿态
        res = self._compute_terrain_pose(self._x, self._y, self._yaw)
        if res is not None:
            z_base, pitch, roll = res
            self._z = z_base
            self._pitch = pitch
            self._roll = roll
            self._last_valid_z = self._z
            self._apply_pose_to_usd()
            print(f"[KINEMATIC] initial pose: z={self._z:.3f}, "
                  f"pitch={math.degrees(pitch):.1f}°, roll={math.degrees(roll):.1f}°")
        else:
            print(f"[KINEMATIC] initial raycast miss, hold z={self._z:.3f}")

    def post_reset(self):
        """与 PolicyController 接口对齐 (run_sim 不直接调用, 兼容用)."""
        self.initialize()

    # ---------- 控制 ----------

    def forward(self, step_size, cmd):
        """每物理步调用一次: 积分 cmd_vel -> SE(3) pose, raycast 拟合地形姿态.

        Args:
            step_size: 物理 dt (秒).
            cmd: array-like, [vx, vy, wz]. 履带车默认忽略 vy.

        姿态:
          - yaw 来自累积控制
          - pitch / roll / z 来自 5 点 raycast 最小二乘拟合的平面 (车头随坡面翘起)
          - pitch / roll / z 用低通滤波平滑, 避免地形噪声引起姿态跳变

        碰撞:
          - 候选 pose (含倾角) 上做 overlap_box, 仅检测车体上半部 (履带通过区不阻挡)
          - 命中则拒绝 xy 更新, yaw 仍可转
        """
        dt = float(step_size)
        vx = float(cmd[0])
        vy = 0.0 if self._ignore_lateral else float(cmd[1])
        wz = float(cmd[2])

        # 0) GUI 拖动同步: 检测 USD translate 是否被用户在 Stage 里手动拖了 (偏差 >1cm).
        # 拖动时**忽略 USD 上的 z** (用户鼠标几乎不可能精确放到地面), 用新 xy 从高空 raycast
        # 强制贴地 + 重置 pitch/roll. 这样用户只关心拖动 xy, z 自动找地面.
        if self._translate_op is not None:
            try:
                usd_pos = self._translate_op.Get()
                if usd_pos is not None:
                    ux, uy, uz = float(usd_pos[0]), float(usd_pos[1]), float(usd_pos[2])
                    if (abs(ux - self._x) > 0.01 or abs(uy - self._y) > 0.01
                            or abs(uz - self._z) > 0.01):
                        self._x, self._y = ux, uy
                        # 从高空 raycast 找地面 (search_height=50m 覆盖大多数场景)
                        r = self._raycast_ground(ux, uy)
                        if r is not None:
                            zh, _nz = r
                            self._z = zh + self._base_link_z
                        else:
                            # raycast 未命中地面, 退回 USD 值, 至少与 GUI 一致
                            self._z = uz
                        self._last_valid_z = self._z
                        self._pitch = 0.0
                        self._roll = 0.0
                        if self._debug_terrain:
                            print(f"[DRAG] sync to ({ux:.2f},{uy:.2f}) -> "
                                  f"raycast_z={'hit' if r else 'miss'} "
                                  f"self._z={self._z:.3f}")
            except Exception:
                pass

        # 1) 积分 yaw
        self._yaw += wz * dt
        if self._yaw > math.pi:
            self._yaw -= 2.0 * math.pi
        elif self._yaw < -math.pi:
            self._yaw += 2.0 * math.pi

        # 2) 候选 xy (车体系 vx 沿前进方向)
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        dx = (vx * c - vy * s) * dt
        dy = (vx * s + vy * c) * dt
        cand_x = self._x + dx
        cand_y = self._y + dy

        # 3) 候选地形姿态 (车头随坡面翘起)
        res = self._compute_terrain_pose(cand_x, cand_y, self._yaw)
        if res is not None:
            target_z, target_pitch, target_roll = res
        else:
            target_z = self._last_valid_z
            target_pitch = self._pitch
            target_roll = self._roll

        # 4) z 直接跟随拟合 (不滤波), 防止上坡瞬态穿模.
        #    pitch / roll 用一阶低通 (alpha=1.0 等效无滤波, 取低值=平滑).
        a = self._attitude_alpha
        new_z = target_z
        new_pitch = (1.0 - a) * self._pitch + a * target_pitch
        new_roll = (1.0 - a) * self._roll + a * target_roll

        # 5) 碰撞盒检测 (用新姿态, 跟随 yaw/pitch/roll)
        if (vx != 0.0 or vy != 0.0) and self._enable_horizontal_collision \
                and self._check_box_overlap(cand_x, cand_y, new_z, self._yaw,
                                            new_pitch, new_roll):
            # 命中障碍 — 保留原 xy/z/pitch/roll, 仅 yaw 更新已生效
            if self._debug_terrain:
                print(f"[BLOCK] xy=({cand_x:.2f},{cand_y:.2f}) "
                      f"pitch={math.degrees(new_pitch):+.1f}° "
                      f"roll={math.degrees(new_roll):+.1f}°")
        else:
            self._x = cand_x
            self._y = cand_y
            self._z = new_z
            self._pitch = new_pitch
            self._roll = new_roll
            if res is not None:
                self._last_valid_z = self._z

        self._apply_pose_to_usd()

    # ---------- 工具 ----------

    def _raycast_ground(self, x, y, search_height: float = 50.0):
        """从 (x, y, +search_height) 向下 raycast 搜索地面 z. 不依赖 self._z (用于 GUI 拖动后强制贴地).

        返回 (z_hit, normal_z) 或 None.
        """
        if self._raycast is None:
            return None
        origin = (float(x), float(y), float(search_height))
        try:
            hit = self._raycast.raycast_closest(
                origin, (0.0, 0.0, -1.0), float(search_height) * 2.0
            )
        except Exception:
            return None
        if not hit or not hit.get("hit", False):
            return None
        pos = hit.get("position")
        if pos is None:
            return None
        normal = hit.get("normal")
        try:
            nz = float(normal[2]) if normal is not None else 1.0
        except Exception:
            nz = 1.0
        return float(pos[2]), nz

    def _raycast_single(self, x, y):
        """在 (x, y) 向下单次 raycast, 返回 (z_hit, normal_z) 或 None.

        起点 z = 当前 base_link.z + raycast_origin_above_base, 向下最多 raycast_max_dist.
        不做法向过滤 — 留给上层聚合阶段处理.
        """
        if self._raycast is None:
            return None
        origin = (
            float(x), float(y),
            float(self._z) + self._raycast_origin_above_base,
        )
        try:
            hit = self._raycast.raycast_closest(
                origin, (0.0, 0.0, -1.0), self._raycast_max_dist
            )
        except Exception as e:
            if not hasattr(self, "_warned_raycast"):
                print(f"[KINEMATIC WARN] raycast exception (suppressed afterwards): {e}")
                self._warned_raycast = True
            return None
        if not hit or not hit.get("hit", False):
            return None
        pos = hit.get("position")
        if pos is None:
            return None
        normal = hit.get("normal")
        try:
            nz = float(normal[2]) if normal is not None else 1.0
        except Exception:
            nz = 1.0
        return float(pos[2]), nz

    def _track_top_z(self, x, y, yaw, cx_b, cy_b, z_cap):
        """模拟一条履带顶面高度: 在履带组中心 (cx_b, cy_b) 沿车体 x 方向子采样若干次,
        每次 raycast 取**最大 z**, 但忽略超过 z_cap 的命中 (障碍物).

        履带物理:
          - 履带横跨多个支撑点, 履带顶面贴在「最高」接触点上 (max 操作).
          - 履带相对当前位置可爬升度有限 (max_climb_height ~ 0.4m). 高出此值的命中
            视为非地形障碍物 (汽车/墙/立柱), 从拟合中剔除. 由 overlap_box 阻挡 xy 前进.

        返回 (max_z_accepted, max_z_rejected, n_miss). max_z_accepted None 表示无可用命中.
        """
        c, s = math.cos(yaw), math.sin(yaw)
        n = self._track_n_sub
        dx = self._track_sub_dx
        offsets = [(cx_b + (i - (n - 1) * 0.5) * dx, cy_b) for i in range(n)]
        max_z = None
        max_rej = None
        n_miss = 0
        for dxb, dyb in offsets:
            wx = x + dxb * c - dyb * s
            wy = y + dxb * s + dyb * c
            r = self._raycast_single(wx, wy)
            if r is None:
                n_miss += 1
                continue
            zh, _nz = r
            if zh > z_cap:
                if max_rej is None or zh > max_rej:
                    max_rej = zh
                continue
            if max_z is None or zh > max_z:
                max_z = zh
        return max_z, max_rej, n_miss

    def _compute_terrain_pose(self, x, y, yaw):
        """履带车 4 履带组模型: 4 组各取最大 z, 拟合平面 -> (z_base, pitch, roll).

        模型:
          - 每条履带顶贴最高支撑点 (跨越凸起, 不陷入侧凹).
          - 4 履带顶 z 拟合平面 z = a*x_b + b*y_b + c0.
          - pitch = -atan2(a, 1)  上坡时 a>0 -> pitch<0 (ROS 中车头抬起对应 pitch<0).
          - roll  =  atan2(b, 1)  左侧高时 b>0 -> roll>0 (左侧抬起).
          - z_base = c0 + base_link_z_above_ground / sqrt(1+a²+b²)
                   (base_link 沿坡面法向位于"履带顶平面"上方 base_link_z).

        相比朴素 5 点平面拟合:
          - 桥前缘垂直面命中不再污染拟合 (max 自动忽略低 z 的侧面命中).
          - 渐进上桥: 前履带先碰到桥面 -> z_f 抬高 -> pitch 增大 -> 车头自然翘起;
            随车前进, 后履带也上桥后 pitch 回归桥面坡度. 完全符合履带车物理.

        返回 (z_base, pitch, roll) 或 None.
        """
        if self._raycast is None:
            return None

        # 可攀爬上限 = 上一帧 base_link z + max_climb. 高出此值的命中视为障碍物.
        # 基准用 base_link.z (而非履带顶 self._z - base_link_z) 让陡坡稳态拟合不被自身
        # 前履带超出阈值: 50° 坡子采样最远点 z 差 ≈ 0.55m * 1.19 = 0.65m, 用 base_link
        # 作基准多 base_link_z=0.20m 余量, max_climb=0.4 就能 cover 到 ~55° 坡.
        z_cap = self._z + self._max_climb_height

        # 4 履带组顶 z + 调试信息
        track_samples = []   # [(x_b, y_b, z_top)]
        dbg_per_track = []   # [(name, accepted, rejected, n_miss)]
        names = ["FL", "FR", "RL", "RR"]
        for (cx_b, cy_b), nm in zip(self._track_centers, names):
            z_top, z_rej, n_miss = self._track_top_z(x, y, yaw, cx_b, cy_b, z_cap)
            dbg_per_track.append((nm, z_top, z_rej, n_miss))
            if z_top is not None:
                track_samples.append((cx_b, cy_b, z_top))

        if len(track_samples) < 3:
            # 拟合失败: debug 打印 (无 throttle, 每次都打)
            if self._debug_terrain:
                detail = " ".join(
                    f"{nm}[acc={'%.3f'%a if a is not None else '--'},"
                    f"rej={'%.3f'%r if r is not None else '--'},miss={m}]"
                    for nm, a, r, m in dbg_per_track
                )
                print(f"[FIT FAIL] self._z={self._z:.3f} z_cap={z_cap:.3f} "
                      f"valid={len(track_samples)}/4 | {detail}")
            return None

        # 最小二乘拟合: A [a b c0]^T = z
        A = np.array([[xb, yb, 1.0] for xb, yb, _ in track_samples], dtype=np.float64)
        Z = np.array([z for _, _, z in track_samples], dtype=np.float64)
        try:
            sol, _resid, _rank, _sv = np.linalg.lstsq(A, Z, rcond=None)
        except Exception:
            return None
        a, b, c0 = float(sol[0]), float(sol[1]), float(sol[2])

        # pitch 取负: ROS REP-103 中 pitch 绕 +Y 轴正向把 +X 转到 -Z (车头朝下).
        # 拟合 a = dz/dx_b > 0 (前方地形高, 上坡), 此时车头应抬起 -> 需要 pitch < 0.
        pitch = -math.atan2(a, 1.0)
        roll = math.atan2(b, 1.0)
        pitch = max(-self._max_tilt, min(self._max_tilt, pitch))
        roll = max(-self._max_tilt, min(self._max_tilt, roll))

        # z_base 沿坡面法向偏移
        norm = math.sqrt(a * a + b * b + 1.0)
        z_base = c0 + self._base_link_z / norm

        # 拟合成功 throttled debug 打印 (60Hz 物理 / 60 = 1Hz)
        if self._debug_terrain:
            self._debug_counter += 1
            if self._debug_counter % 60 == 0:
                detail = " ".join(
                    f"{nm}[acc={'%.3f'%a_ if a_ is not None else '--'},"
                    f"rej={'%.3f'%r_ if r_ is not None else '--'}]"
                    for nm, a_, r_, _m in dbg_per_track
                )
                print(f"[FIT OK] z_base={z_base:.3f} pitch={math.degrees(pitch):+.1f}° "
                      f"roll={math.degrees(roll):+.1f}° z_cap={z_cap:.3f} | {detail}")
        return z_base, pitch, roll

    def _check_box_overlap(self, x, y, z, yaw, pitch=0.0, roll=0.0):
        """在 (x, y, z, yaw, pitch, roll) pose 上做 PhysX overlap_box 检测.

        关键设计:
          1. 碰撞盒**只覆盖 chassis 上半** (默认 box_h = size_z/2 = 0.15m).
             下半视为履带通过区, 凹凸/坡面/桥前缘的垂直探入不阻挡.
          2. 碰撞盒**完整跟随车体姿态 (yaw + pitch + roll)**.
             仅覆盖上半部时, 前下角在车体系 (hx,0,0), 绕 y 旋转 pitch 后世界 z 抬高
             hx·sin|pitch|, 与前方坡面抬高量 hx·tan|pitch| 在小角下基本相等, 不会
             误嵌坡面. 反之保持水平的碰撞盒底沿会被前方坡面探入 -> 卡死在坡前.

        命中条件: 任意有 CollisionAPI 的 prim 与碰撞盒重叠. chassis_visual 无 collider,
        不会自我命中. 适合阻挡墙/栈桥栏杆/立柱等高出车体底部的实体障碍.
        """
        if self._raycast is None:
            return False

        # 碰撞盒高度独立配置 (不再耦合 size_z), 默认 0.5m 用来撞栏杆 / 立柱.
        # 底沿抬高到 base_link.z + collision_box_bottom_above_base (默认 = max_climb_height),
        # 让 max_climb 以下的台阶 / 凸起视作履带通过区, 不撞碰撞盒.
        box_h = max(float(self._collision_box_height), 0.02)
        hx = float(self._size[0]) * 0.5 * self._collision_shrink
        hy = float(self._size[1]) * 0.5 * self._collision_shrink
        hz = box_h * 0.5 * self._collision_shrink

        # 车体系中心 z 偏移 (沿 base_link local z 轴向上)
        h_off_z = self._collision_box_bottom_above_base + box_h * 0.5

        # base_link local z 轴在世界系下的方向 (ZYX: yaw,pitch,roll 内禀欧拉的 R[:,2] 列)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        # local z 在世界系: (sin(pitch), -sin(roll)*cos(pitch), cos(pitch)*cos(roll))
        ux, uy, uz = sp, -sr * cp, cp * cr
        center_x = float(x) + h_off_z * ux
        center_y = float(y) + h_off_z * uy
        center_z = float(z) + h_off_z * uz

        qw, qx, qy, qz = self._euler_to_quat(yaw, pitch, roll)
        # PhysX overlap_box 期望 quat 顺序 (x, y, z, w)
        rot = (qx, qy, qz, qw)
        pos = (center_x, center_y, center_z)

        self._overlap_hit_flag = False

        def _report(hit):
            self._overlap_hit_flag = True
            return False

        try:
            self._raycast.overlap_box(
                (hx, hy, hz), pos, rot, _report, False
            )
        except Exception as e:
            if not hasattr(self, "_warned_overlap"):
                print(f"[KINEMATIC WARN] overlap_box exception (suppressed afterwards): {e}")
                self._warned_overlap = True
            return False

        return self._overlap_hit_flag

    @staticmethod
    def _euler_to_quat(yaw, pitch, roll):
        """ZYX (yaw -> pitch -> roll) 内禀欧拉到四元数 (w, x, y, z).

        ROS REP-103 约定: x 前, y 左, z 上.
        roll  = 绕 x 轴旋转 (右翻为正)
        pitch = 绕 y 轴旋转 (车头抬起为正)
        yaw   = 绕 z 轴旋转 (左转为正)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qw, qx, qy, qz

    def _apply_pose_to_usd(self):
        """把内部 (x, y, z, yaw, pitch, roll) 写到 USD Xform."""
        if self._translate_op is None or self._orient_op is None:
            return
        self._translate_op.Set(Gf.Vec3d(self._x, self._y, self._z))
        qw, qx, qy, qz = self._euler_to_quat(self._yaw, self._pitch, self._roll)
        self._orient_op.Set(Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))

    def _apply_pose(self, x, y, z, qw, qx, qy, qz):
        """外部 (如 reset_robot) 直接 set pose: 同步内部状态 + USD.

        重置时把 pitch / roll 清零, 下一帧 forward() 会重新从地形拟合得到.
        """
        self._x = float(x)
        self._y = float(y)
        self._z = float(z)
        self._yaw = _yaw_from_quat_wxyz(qw, qx, qy, qz)
        self._pitch = 0.0
        self._roll = 0.0
        self._last_valid_z = self._z
        self._apply_pose_to_usd()

    def _reset_motion_state(self):
        """clear cmd 缓存 (运动学模式下并无, 占位)."""
        return
