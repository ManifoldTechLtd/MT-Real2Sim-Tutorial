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

simulation_app = SimulationApp({"headless": headless})
print(f"[run_sim] SimulationApp started (headless={headless})")

# ---- SimulationApp 启动后, 插入项目路径 ----
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

# ---- 导入依赖 ----
import numpy as np
from pxr import Usd, UsdGeom

from r2s2r.core.scene_builder import load_config
from r2s2r.core.robot_factory import build_quadruped_scene
from r2s2r.bridge.tcp_server import TcpBridgeServer

# ---- 加载配置 ----
config = load_config(config_path)
print(f"[run_sim] Config loaded: {config_path}")

# ---- 创建 World ----
from isaacsim.core.api import World

physics_dt = 1.0 / 500.0  # 四足 RL 策略标准频率
rendering_dt = 1.0 / 50.0  # 渲染 50Hz

world = World(
    stage_units_in_meters=1.0,
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

# ---- 搭建场景 ----
robot, suite = build_quadruped_scene(world, config)
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


# ---- World 回调 ----
import omni.usd
_stage = omni.usd.get_context().get_stage()

# 查找 body prim
quad_cfg = config["quadruped"]
_body_link = {"spot": "body", "go2": "base"}.get(quad_cfg.get("robot_type", "spot"), "body")
_body_prim_path = f"{quad_cfg.get('prim_path', '/World/Quadruped')}/{_body_link}"
_body_prim = _stage.GetPrimAtPath(_body_prim_path)

# 初始位姿 (用于复位)
_spawn_pos = np.array(quad_cfg.get("position", [0, 0, 0.5]), dtype=np.float32)
_spawn_ori = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)  # quat (x,y,z,w), 无旋转

# 控制参数
_ctrl = config.get("control", {})
_lin_speed = _ctrl.get("linear_speed", 3.0)
_lat_speed = _ctrl.get("lateral_speed", 3.0)
_yaw_speed = _ctrl.get("yaw_speed", 3.0)

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
_last_pc_time = 0.0
_last_img_time = 0.0
_last_imu_time = 0.0


def on_physics_step(step_size):
    """物理回调 (500Hz): RL 策略前向推理"""
    global _cmd, _need_reinit
    if _need_reinit:
        _need_reinit = False
        robot.initialize()
    robot.forward(step_size, _cmd)


def on_render_step():
    """渲染回调 (~50Hz): TCP 桥接发布 + 指令接收"""
    global _cmd, _frame_count, _last_pose_time, _last_pc_time, _last_img_time, _last_imu_time, _ever_received_ros_cmd

    _frame_count += 1
    now = time.time()

    # ---- 检查 RESET 请求 (来自 ROS1) ----
    if bridge_server.get_reset_requested():
        _reset_robot()

    # ---- 接收 cmd_vel (来自 ROS1) ----
    # 优先级: ROS cmd_vel > 键盘 (键盘已在 update_keyboard_cmd 中设置 _cmd)
    ros_cmd = bridge_server.get_latest_cmd_vel(timeout=1.0)
    if ros_cmd is not None:
        _ever_received_ros_cmd = True
        _cmd[0] = ros_cmd[0]  # vx — 覆盖键盘值
        _cmd[1] = ros_cmd[1]  # vy
        _cmd[2] = ros_cmd[2]  # wz
        if ros_cmd[0] != 0 or ros_cmd[1] != 0 or ros_cmd[2] != 0:
            print(f"[run_sim] ROS cmd → _cmd: vx={ros_cmd[0]:.2f} vy={ros_cmd[1]:.2f} wz={ros_cmd[2]:.2f}")
    elif bridge_server.connected:
        if _frame_count % 100 == 0:
            print(f"[run_sim] get_latest_cmd_vel=None, conn=True, ever_recv={_ever_received_ros_cmd}")
        if _ever_received_ros_cmd:
            # 之前收到过 ROS cmd_vel 但超时 → 安全停车
            if _cmd[0] != 0 or _cmd[1] != 0 or _cmd[2] != 0:
                print("[run_sim] cmd_vel timeout → safety stop")
            _cmd[:] = 0.0
    # else: 从未收到 ROS cmd 或 bridge 未连接 → 保留键盘 _cmd

    # ---- 发布 POSE ----
    if now - _last_pose_time >= _pose_interval and _body_prim.IsValid():
        mat = get_robot_world_transform(_body_prim)
        x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
        # 从 3x3 旋转矩阵提取完整四元数 (保留 roll/pitch/yaw)
        qx, qy, qz, qw = mat3_to_quat(mat[:3, :3])
        bridge_server.send_pose(
            x, y, z, qx, qy, qz, qw,
            vx=_cmd[0], vy=_cmd[1], wz=_cmd[2],
            timestamp=now,
        )
        _last_pose_time = now

    # ---- 发布 POINTCLOUD ----
    if now - _last_pc_time >= _pc_interval and suite is not None:
        lidar_data = suite.get_data_by_type("lidar")
        for name, data in lidar_data.items():
            pc = data.get("point_cloud")
            if pc is not None and len(pc) > 0:
                # LiDAR 点云在 sensor local frame, 转换到世界坐标
                if _body_prim.IsValid():
                    T = get_robot_world_transform(_body_prim)
                    R = T[:3, :3]
                    t = T[:3, 3]
                    pc_world = (R @ pc.T).T + t
                    bridge_server.send_pointcloud(pc_world, frame_id="map", timestamp=now)
        _last_pc_time = now

    # ---- 发布 IMAGE ----
    if now - _last_img_time >= _img_interval and suite is not None:
        camera_data = suite.get_data_by_type("camera")
        for name, data in camera_data.items():
            rgb = data.get("rgb")
            if rgb is not None and rgb.size > 0:
                bridge_server.send_image(rgb, frame_id=name,
                                         quality=_img_quality, timestamp=now)
        _last_img_time = now

    # ---- 发布 IMU ----
    if now - _last_imu_time >= _imu_interval and suite is not None:
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
    global _cmd
    if not _has_keyboard:
        return
    # 不再因 bridge 连接就禁用键盘!
    # 键盘始终设置 _cmd, 如果 ROS cmd_vel 到达则在 on_render_step 中覆盖

    vx, vy, wz = 0.0, 0.0, 0.0
    import carb.input
    K = carb.input.KeyboardInput
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

    # R 键: 复位机器人到初始位置
    if _key_state.get(K.R, False):
        _key_state[K.R] = False
        _reset_robot()

    # P 键: 截图保存
    if _key_state.get(K.P, False):
        _key_state[K.P] = False
        _save_screenshot()


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
    global _cmd
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
    global _frame_count

    # 启动 TCP 桥接
    bridge_server.start()

    # World 初始化
    world.reset()
    robot.initialize()
    if suite:
        suite.initialize()
    print("[run_sim] World reset, robot and sensors initialized")

    # 注册物理回调 (必须在 play/pause 之前)
    world.add_physics_callback("rl_policy", on_physics_step)

    # 设置键盘 (必须在 world 初始化后)
    _setup_keyboard()

    # 管理仿真播放状态 (关键! 参考 01_quadruped_nav.py)
    if headless:
        world.play()
        print("[run_sim] Headless mode → auto play")
    else:
        world.pause()
        print("[run_sim] GUI mode → paused, click PLAY in Isaac Sim to start")

    print("=" * 60)
    print(f"[run_sim] TCP bridge: {bridge_cfg.get('host', '0.0.0.0')}:{bridge_cfg.get('port', 9090)}")
    print(f"[run_sim] Robot: {config['quadruped']['robot_type']}")
    print(f"[run_sim] Keyboard: WASD+QE=move, R=reset, P=screenshot (click viewport first)")
    print(f"[run_sim] GUI mode: click [▶ PLAY] button in Isaac Sim to start")
    print("=" * 60)

    _diag_time = time.time()

    try:
        while simulation_app.is_running():
            world.step(render=True)

            # 等待用户点击 PLAY
            if not world.is_playing():
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
        bridge_server.stop()
        print("[run_sim] Bridge stopped")

    simulation_app.close()
    print("[run_sim] Done")


if __name__ == "__main__":
    main()
