"""
run_sim.py
Isaac Sim 仿真主入口 — 启动仿真 + TCP 桥接服务端

用法:
    ./tools/run_isaac.sh r2s2r/run_sim.py --config r2s2r/configs/config.yaml
"""

# ---- 预加载 (必须在 SimulationApp 之前) ----
import argparse
import os
import sys
import math
import time

# ---- CLI 参数 ----
parser = argparse.ArgumentParser(description="Isaac Real2Sim2Real Simulation")
parser.add_argument("--config", type=str, default="r2s2r/configs/config.yaml",
                    help="仿真配置文件路径 (默认: r2s2r/configs/config.yaml)")
parser.add_argument("--headless", action="store_true",
                    help="无头模式 (覆盖配置文件)")
args = parser.parse_args()

# ---- 启动 SimulationApp ----
from isaacsim import SimulationApp

config_path = os.path.abspath(args.config)
# 临时加载 config 获取 headless 设置
import yaml
with open(config_path) as f:
    _cfg = yaml.safe_load(f)
headless = args.headless or _cfg.get("simulation", {}).get("headless", False)
_sim_cfg = _cfg.get("simulation", {})
app_settings = {"headless": headless}
if not headless:
    win_w = _sim_cfg.get("window_width")
    win_h = _sim_cfg.get("window_height")
    if win_w and win_h:
        app_settings["width"] = int(win_w)
        app_settings["height"] = int(win_h)

simulation_app = SimulationApp(app_settings)
print(f"[run_sim] SimulationApp started (headless={headless}, "
      f"window={app_settings.get('width', 'default')}x{app_settings.get('height', 'default')})")

# ---- SimulationApp 启动后, 插入项目路径 ----
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# ---- 导入依赖 ----
import numpy as np
from pxr import Usd, UsdGeom

from r2s2r.core.scene_builder import load_config
from r2s2r.core.robot_factory import build_quadruped_scene, physics_dt_for
from r2s2r.bridge.tcp_server import TcpBridgeServer
from r2s2r.core.observation import filter_body_returns, fresh_scan, fresh_camera_frame
from r2s2r.core.log_rate import log_throttled

# ---- 加载配置 ----
config = load_config(config_path)
print(f"[run_sim] Config loaded: {config_path}")

# 环境变量覆盖
robot_type_env = os.getenv('ROBOT_TYPE')
if robot_type_env:
    config['quadruped']['robot_type'] = robot_type_env
    print(f"[config] ROBOT_TYPE overridden by env: {robot_type_env}")

scene_path_env = os.getenv('SCENE_PATH')
if scene_path_env:
    if 'scene' not in config:
        config['scene'] = {}
    if 'environment' not in config['scene']:
        config['scene']['environment'] = {}
    config['scene']['environment']['usdz_path'] = scene_path_env
    print(f"[config] SCENE_PATH overridden by env: {scene_path_env}")

collision_approx_env = os.getenv('COLLISION_APPROX')
if collision_approx_env:
    config.setdefault('scene', {}).setdefault('physics', {})['collision_approximation'] = collision_approx_env
    print(f"[config] COLLISION_APPROX overridden by env: {collision_approx_env}")

contact_offset_env = (os.getenv('CONTACT_OFFSET') or '').strip()
if contact_offset_env:
    config.setdefault('scene', {}).setdefault('physics', {})['contact_offset'] = float(contact_offset_env)
    print(f"[config] CONTACT_OFFSET overridden by env: {contact_offset_env}")

# ---- 创建 World ----
from isaacsim.core.api import World

physics_dt = physics_dt_for(config["quadruped"].get("robot_type", ""))
render_hz = float(config.get("simulation", {}).get("render_hz", 50.0))
rendering_dt = 1.0 / render_hz
print(f"[run_sim] physics_dt={physics_dt:.4f}, rendering_dt={rendering_dt:.4f} ({render_hz:g}Hz)")

sim_params = {}
gpu_parts_env = (os.getenv('GPU_MAX_PARTITIONS') or '').strip()
if gpu_parts_env:
    sim_params['gpu_max_num_partitions'] = int(gpu_parts_env)
    print(f"[config] GPU_MAX_PARTITIONS={gpu_parts_env}")

world = World(
    stage_units_in_meters=1.0,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
    device=os.getenv('PHYSX_DEVICE') or None,
    sim_params=sim_params or None,
)
if os.getenv('PHYSX_DEVICE'):
    print(f"[config] PHYSX_DEVICE={os.getenv('PHYSX_DEVICE')}")

gpu_dyn_env = (os.getenv('PHYSX_GPU_DYNAMICS') or '').strip()
if gpu_dyn_env:
    enable = gpu_dyn_env.lower() not in ('0', 'false', 'off', 'no')
    world.get_physics_context().enable_gpu_dynamics(flag=enable)
    print(f"[config] PHYSX_GPU_DYNAMICS={gpu_dyn_env} -> enable_gpu_dynamics({enable})")

# ---- 搭建场景 ----
robot, suite, body_prim_path = build_quadruped_scene(world, config)
print("[run_sim] Scene built")

# ---- TCP 桥接服务端 ----
bridge_cfg = config.get("bridge", {})
bridge_server = TcpBridgeServer(
    host=bridge_cfg.get("host", "0.0.0.0"),
    port=bridge_cfg.get("port", 9090),
)

# ---- 辅助函数 ----

def get_robot_pose(body_prim) -> tuple:
    """获取机器人世界位姿: (pos_xy, yaw, z)"""
    usd_time = Usd.TimeCode.Default()
    xform = UsdGeom.Imageable(body_prim).ComputeLocalToWorldTransform(usd_time)
    mat = np.array(xform, dtype=np.float64).T
    x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
    yaw = math.atan2(mat[1, 0], mat[0, 0])
    return np.array([x, y], dtype=np.float64), yaw, z


def get_robot_world_transform(body_prim) -> np.ndarray:
    """获取机器人 4x4 世界变换矩阵"""
    usd_time = Usd.TimeCode.Default()
    xform = UsdGeom.Imageable(body_prim).ComputeLocalToWorldTransform(usd_time)
    return np.array(xform, dtype=np.float64).T


def mat3_to_quat(R: np.ndarray) -> tuple:
    """3x3 旋转矩阵 → 四元数 (qx, qy, qz, qw), Shepperd 方法, 数值稳定"""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def pose_twist(previous, mat, stamp):
    """Differentiate published poses in simulation time; return body-frame twist."""
    linear, angular = np.zeros(3), np.zeros(3)
    if previous is None or stamp <= previous[1]:
        return linear, angular
    old, old_stamp = previous
    dt = stamp - old_stamp
    rotation = mat[:3, :3]
    linear = rotation.T @ (mat[:3, 3] - old[:3, 3]) / dt
    q = np.array(mat3_to_quat(rotation @ old[:3, :3].T))
    if q[3] < 0.0:
        q = -q
    sine = np.linalg.norm(q[:3])
    if sine > 1e-12:
        angular = rotation.T @ q[:3] * (2.0 * math.atan2(sine, q[3]) / (sine * dt))
    return linear, angular


# ---- World 回调 ----
import omni.usd
_stage = omni.usd.get_context().get_stage()

# 查找 body prim (从 robot_factory 统一获取, 不自建映射表)
_body_prim = _stage.GetPrimAtPath(body_prim_path)

# 初始位姿 (用于复位)
quad_cfg = config["quadruped"]
_spawn_pos = np.array(quad_cfg.get("position", [0, 0, 0.5]), dtype=np.float32)
_spawn_ori = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)  # quat (x,y,z,w), 无旋转

# 控制参数
_ctrl = config.get("control", {})
_lin_speed = _ctrl.get("linear_speed", 3.0)
_lat_speed = _ctrl.get("lateral_speed", 3.0)
_yaw_speed = _ctrl.get("yaw_speed", 3.0)

# 键盘边沿检测
_r_prev = False  # R 键: 原地复位
_p_prev = False  # P 键: 回起点

# 速度指令状态
_cmd = np.zeros(3, dtype=np.float64)  # [vx, vy, wz]
_frame_count = 0
_need_reinit = False
_ever_received_ros_cmd = False  # 是否曾收到过 ROS cmd_vel

# 发布频率控制
_pose_interval = 1.0 / bridge_cfg.get("pose_rate", 50)
_pc_interval = 1.0 / bridge_cfg.get("pointcloud_rate", 10)
_img_interval = 1.0 / bridge_cfg.get("image_rate", 10)
_imu_interval = 1.0 / bridge_cfg.get("imu_rate", 50)
_img_quality = bridge_cfg.get("image_quality", 80)
_last_pose_time = 0.0
_previous_pose_sample = None
_last_pc_time = 0.0
_last_img_time = 0.0
_last_imu_time = 0.0
_last_scan_ns = {}
_last_camera_stamps = {}
_observation_epoch = world.current_time
_last_scan_warning = 0.0

# Filter the configured body box using the measured scan pose.
_robot_type = config["quadruped"].get("robot_type", "")
_sf_cfg = config.get("kinematic_car", {}).get("lidar_self_filter", {})
_self_filter_enabled = _robot_type == "kinematic_car" and bool(_sf_cfg.get("enable", False))
_self_filter_margin = float(_sf_cfg.get("margin", 0.01))
if not math.isfinite(_self_filter_margin) or _self_filter_margin < 0:
    raise ValueError("invalid lidar self-filter margin")


def reset_robot(to_start: bool = False):
    """复位机器人姿态 (移植自 tunnel-inspection, 生产验证)

    Args:
        to_start:
            False (R键/原地恢复): 保留当前 (x, y, yaw), 抬升 z 避免穿模, pitch/roll 清零,
                                 关节回默认站立, 速度清零. 用于侧翻/卡障碍原地恢复.
            True  (P键/回起点):   强制回到 config.yaml `quadruped.position` 起点, yaw=0,
                                 其余清零. 用于反复测试回原点.
    """
    global _need_reinit, _previous_pose_sample, _observation_epoch
    _previous_pose_sample = None
    _observation_epoch = world.current_time
    _last_camera_stamps.clear()
    if suite is not None:
        for writer in suite._camera_writers.values():
            writer.latest = {}
    try:
        pos, quat = robot.robot.get_world_pose()
        if pos is None or quat is None:
            print("[RESET] pose unavailable")
            return

        if to_start:
            # 回起点: 强制 xy + yaw=0
            sx, sy, sz = _spawn_pos
            x, y = float(sx), float(sy)
            z = max(float(sz), 0.5)
            yaw = 0.0
        else:
            # 原地恢复: 保留 xy/yaw, 抬升 z, 清 roll/pitch
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            # Isaac Sim quat order: (w, x, y, z)
            w, qx, qy, qz = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            yaw = math.atan2(2.0 * (w * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
            z = max(z, 0.5)

        # 新姿态: 仅 yaw, 无 roll/pitch
        new_quat = np.array(
            [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
            dtype=np.float64,
        )
        new_pos = np.array([x, y, z], dtype=np.float64)

        robot.robot.set_world_pose(new_pos, new_quat)
        robot.robot.set_linear_velocity(np.zeros(3))
        robot.robot.set_angular_velocity(np.zeros(3))

        # 关节复位 (四足机器人有 default_pos, kinematic_car 为 None)
        if hasattr(robot, "default_pos") and robot.default_pos is not None:
            n_dof = len(robot.default_pos)
            try:
                robot.robot.set_joint_positions(robot.default_pos)
                robot.robot.set_joint_velocities(np.zeros(n_dof))
            except Exception:
                # 部分 Articulation 需用 _articulation_view 调用 (兼容 fallback)
                try:
                    robot.robot._articulation_view.set_joint_positions(
                        robot.default_pos.reshape(1, -1)
                    )
                    robot.robot._articulation_view.set_joint_velocities(
                        np.zeros((1, n_dof))
                    )
                except Exception as ex2:
                    print(f"[RESET WARN] joint reset fallback failed: {ex2}")

        _need_reinit = True  # 触发下次物理步 robot.initialize()
        tag = "START" if to_start else "INPLACE"
        print(f"[RESET {tag}] pos=({x:.2f}, {y:.2f}, {new_pos[2]:.2f}) "
              f"yaw={math.degrees(yaw):.1f}deg")
    except Exception as e:
        print(f"[RESET WARN] {e}")


def on_physics_step(step_size):
    """物理回调 (每个物理步): RL 策略前向推理"""
    global _cmd, _need_reinit
    if _need_reinit:
        _need_reinit = False
        robot.initialize()
    robot.forward(step_size, _cmd)


def on_render_step():
    """渲染回调 (~50Hz): TCP 桥接发布 + 指令接收"""
    global _cmd, _frame_count, _last_pose_time, _last_pc_time, _last_img_time, _last_imu_time, _ever_received_ros_cmd, _previous_pose_sample
    global _last_scan_warning

    _frame_count += 1
    now = world.current_time
    bridge_server.send_clock(now)

    # ---- 检查 RESET 请求 (来自 ROS1) ----
    reset_req = bridge_server.get_reset_requested()
    if reset_req is not None:
        reset_robot(to_start=reset_req)

    # ---- 接收 cmd_vel (来自 ROS1) ----
    # 优先级: ROS cmd_vel > 键盘 (键盘已在 update_keyboard_cmd 中设置 _cmd)
    ros_cmd = bridge_server.get_latest_cmd_vel(timeout=1.0)
    if ros_cmd is not None:
        _ever_received_ros_cmd = True
        _cmd[0] = ros_cmd[0]  # vx — 覆盖键盘值
        _cmd[1] = ros_cmd[1]  # vy
        _cmd[2] = ros_cmd[2]  # wz
        if ros_cmd[0] != 0 or ros_cmd[1] != 0 or ros_cmd[2] != 0:
            log_throttled("ros_cmd",
                          f"[run_sim] ROS cmd → _cmd: vx={ros_cmd[0]:.2f} vy={ros_cmd[1]:.2f} wz={ros_cmd[2]:.2f}")
    elif bridge_server.connected:
        log_throttled("no_cmd",
                      f"[run_sim] get_latest_cmd_vel=None, conn=True, ever_recv={_ever_received_ros_cmd}")
        if _ever_received_ros_cmd:
            # 之前收到过 ROS cmd_vel 但超时 → 安全停车
            if _cmd[0] != 0 or _cmd[1] != 0 or _cmd[2] != 0:
                print("[run_sim] cmd_vel timeout → safety stop")
            _cmd[:] = 0.0
    # else: 从未收到 ROS cmd 或 bridge 未连接 → 保留键盘 _cmd

    # ---- 发布 POSE ----
    if now - _last_pose_time + 1e-9 >= _pose_interval and _body_prim.IsValid():
        mat = get_robot_world_transform(_body_prim)
        x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
        # 从 3x3 旋转矩阵提取完整四元数 (保留 roll/pitch/yaw)
        qx, qy, qz, qw = mat3_to_quat(mat[:3, :3])
        linear, angular = pose_twist(_previous_pose_sample, mat, now)
        bridge_server.send_pose(
            x, y, z, qx, qy, qz, qw,
            vx=linear[0], vy=linear[1], vz=linear[2],
            wx=angular[0], wy=angular[1], wz=angular[2],
            timestamp=now,
        )
        _previous_pose_sample = (mat.copy(), now)
        _last_pose_time = now

    # ---- 发布 POINTCLOUD ----
    if now - _last_pc_time + 1e-9 >= _pc_interval and suite is not None:
        lidar_data = suite.get_data_by_type("lidar")
        for name, data in lidar_data.items():
            if 'error' in data:
                if now-_last_scan_warning > 2.0:
                    print(f"[LIDAR] rejected observation: {data['error']}")
                    _last_scan_warning = now
                continue
            if not fresh_scan(data, _last_scan_ns.get(name, -1), now, _observation_epoch):
                continue
            _last_scan_ns[name] = data['scan_stamp_ns']
            pc_world = data.get("point_cloud")
            if pc_world is not None and len(pc_world) > 0:
                body_lidar = suite.sensor_to_body(name)
                body_world = data['lidar_to_world'] @ np.linalg.inv(body_lidar)
                if _self_filter_enabled:
                    pc_world = filter_body_returns(pc_world, data['lidar_to_world'], body_lidar,
                        config['kinematic_car']['size'], _self_filter_margin,
                        body_shape=config['kinematic_car'].get('body_shape', 'box'),
                        cylinder_radius=config['kinematic_car'].get('cylinder_radius', .25),
                        cylinder_offset=config['kinematic_car'].get('cylinder_offset', .18))
                if len(pc_world) == 0:
                    continue
                pose = [*body_world[:3,3].tolist(), *mat3_to_quat(body_world[:3,:3])]
                bridge_server.send_pointcloud(
                    pc_world, frame_id="map", timestamp=data['measurement_stamp'], body_pose=pose)
        _last_pc_time = now

    # ---- 发布 IMAGE ----
    if now - _last_img_time + 1e-9 >= _img_interval and suite is not None:
        camera_data = suite.get_data_by_type("camera")
        for name, data in camera_data.items():
            if not fresh_camera_frame(data, _last_camera_stamps.get(name, -1),
                                      now, _observation_epoch):
                continue
            rgb = data.get("rgb")
            if rgb is not None and rgb.size > 0:
                body_cam = suite.sensor_to_body(name)
                bridge_server.send_image(
                    rgb, frame_id=name, quality=_img_quality,
                    timestamp=data['measurement_stamp'],
                    intrinsics=suite.camera_intrinsics(name),
                    body_to_camera=(body_cam[:3, 3].tolist(),
                                    mat3_to_quat(body_cam[:3, :3])))
                _last_camera_stamps[name] = data['measurement_stamp']
        _last_img_time = now

    # ---- 发布 IMU ----
    if now - _last_imu_time + 1e-9 >= _imu_interval and suite is not None:
        imu_data = suite.get_data_by_type("imu")
        for name, data in imu_data.items():
            lin_acc = data.get("lin_acc")
            ang_vel = data.get("ang_vel")
            orientation = data.get("orientation")
            if lin_acc is not None and ang_vel is not None:
                bridge_server.send_imu(lin_acc, ang_vel, orientation,
                                       frame_id=name, timestamp=now)
        _last_imu_time = now


# ---- 键盘输入 (调试用, 无 ROS 时可用) ----
_key_state = {}
_has_keyboard = False

def _setup_keyboard():
    """在 World 初始化后设置键盘输入 (Isaac Sim 5.x omni.appwindow API)"""
    global _has_keyboard
    try:
        import carb.input
        import omni.appwindow

        app_window = omni.appwindow.get_default_app_window()
        keyboard = app_window.get_keyboard()
        input_iface = carb.input.acquire_input_interface()

        def _on_kb_event(event, *args, **kwargs):
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                _key_state[event.input] = True
            elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
                _key_state[event.input] = False
            return True

        input_iface.subscribe_to_keyboard_events(keyboard, _on_kb_event)
        _has_keyboard = True
        print("[run_sim] Keyboard input enabled (WASD+QE) via omni.appwindow")
    except Exception as e:
        _has_keyboard = False
        print(f"[run_sim] Keyboard input not available: {e}")


def update_keyboard_cmd():
    """从键盘状态更新速度指令 (作为 fallback, ROS cmd_vel 在 on_render_step 中覆盖)"""
    global _cmd, _r_prev, _p_prev
    if not _has_keyboard:
        return
    # 不再因 bridge 连接就禁用键盘!
    # 键盘始终设置 _cmd, 如果 ROS cmd_vel 到达则在 on_render_step 中覆盖

    import carb.input
    K = carb.input.KeyboardInput

    # R 键边沿检测: 原地复位 (保留 xy/yaw, 清姿态)
    r_now = _key_state.get(K.R, False)
    if r_now and not _r_prev:
        reset_robot(to_start=False)
    _r_prev = r_now

    # P 键边沿检测: 回起点 (xy/yaw 强制为 config 起点, yaw=0)
    p_now = _key_state.get(K.P, False)
    if p_now and not _p_prev:
        reset_robot(to_start=True)
    _p_prev = p_now

    vx, vy, wz = 0.0, 0.0, 0.0
    if _key_state.get(K.W, False):
        vx += _lin_speed
    if _key_state.get(K.S, False):
        vx -= _lin_speed
    if _key_state.get(K.A, False):
        vy += _lat_speed
    if _key_state.get(K.D, False):
        vy -= _lat_speed
    if _key_state.get(K.Q, False):
        wz += _yaw_speed
    if _key_state.get(K.E, False):
        wz -= _yaw_speed
    if vx != _cmd[0] or vy != _cmd[1] or wz != _cmd[2]:
        print(f"[run_sim] KB cmd: vx={vx:.1f} vy={vy:.1f} wz={wz:.1f}")
    _cmd[0] = vx
    _cmd[1] = vy
    _cmd[2] = wz


def _save_screenshot():
    """保存当前相机画面到 output/ 目录"""
    if suite is None:
        print("[run_sim] No sensor suite, cannot screenshot")
        return
    import cv2
    from datetime import datetime
    camera_data = suite.get_data_by_type("camera")
    if not camera_data:
        print("[run_sim] No camera data available")
        return
    out_dir = os.path.join(_project_root, "output")
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    for name, data in camera_data.items():
        rgb = data.get("rgb")
        if rgb is not None and rgb.size > 0:
            path = os.path.join(out_dir, f"{name}_{ts}.png")
            cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            print(f"[run_sim] Screenshot saved: {path}")


def _reset_robot():
    """将机器人传送回初始出生点, 清零速度"""
    global _cmd, _previous_pose_sample, _observation_epoch
    _previous_pose_sample = None
    _observation_epoch = world.current_time
    _last_camera_stamps.clear()
    if suite is not None:
        for writer in suite._camera_writers.values():
            writer.latest = {}
    try:
        robot.robot.set_world_pose(position=_spawn_pos, orientation=_spawn_ori)
        robot.robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
        robot.robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
        _cmd[:] = 0.0
        print(f"[run_sim] Robot RESET to {_spawn_pos.tolist()}")
    except Exception as e:
        print(f"[run_sim] Reset failed: {e}")


# ---- 主循环 ----

def main():
    global _frame_count, _previous_pose_sample, _observation_epoch, _last_img_time
    suite_initialized = False

    try:
        # 启动 TCP 桥接
        bridge_server.start()

        # World 初始化
        world.reset()
        robot.initialize()
        if suite:
            suite.initialize()
            suite_initialized = True
        print("[run_sim] World reset, robot and sensors initialized")

        # 注册物理回调 (必须在 play/pause 之前)
        world.add_physics_callback("rl_policy", on_physics_step)

        # 设置键盘 (必须在 world 初始化后)
        _setup_keyboard()

        # 管理仿真播放状态 (关键! 参考 01_quadruped_nav.py)
        # 自动开始播放（headless 和 GUI 模式都自动开始）
        world.play()
        print("[run_sim] Auto play enabled")

        print("=" * 60)
        print(f"[run_sim] TCP bridge: {bridge_cfg.get('host', '0.0.0.0')}:{bridge_cfg.get('port', 9090)}")
        print(f"[run_sim] Robot: {config['quadruped']['robot_type']}")
        print(f"[run_sim] Keyboard: WASD+QE=move, R=reset, P=screenshot (click viewport first)")
        print(f"[run_sim] GUI mode: click [▶ PLAY] button in Isaac Sim to start")
        print("=" * 60)

        _diag_time = time.time()

        while simulation_app.is_running():
            world.step(render=True)

            # 等待用户点击 PLAY
            if not world.is_playing():
                _previous_pose_sample = None
                _observation_epoch = world.current_time
                _last_camera_stamps.clear()
                _last_img_time = 0.0
                if suite_initialized:
                    for writer in suite._camera_writers.values():
                        writer.latest = {}
                continue

            # 检测 stop → play 重新初始化
            if world.is_stopped():
                _need_reinit = True

            update_keyboard_cmd()
            on_render_step()

            # 周期性诊断日志 (每 5 秒)
            now_diag = time.time()
            if now_diag - _diag_time >= 5.0:
                conn_str = 'ROS' if bridge_server.connected else 'NONE'
                kb_str = 'ON' if _has_keyboard else 'OFF'
                print(f"[diag] frame={_frame_count} conn={conn_str} kb={kb_str} "
                      f"cmd=[{_cmd[0]:+.2f},{_cmd[1]:+.2f},{_cmd[2]:+.2f}]")
                _diag_time = now_diag

    except KeyboardInterrupt:
        print("\n[run_sim] Interrupted")
    finally:
        try:
            bridge_server.stop()
            print("[run_sim] Bridge stopped")
        finally:
            try:
                if suite_initialized:
                    suite.close()
            finally:
                simulation_app.close()
    print("[run_sim] Done")


if __name__ == "__main__":
    main()
