"""
sensor_suite.py
传感器套件模块 - 通过内外参配置文件组合 Camera + LiDAR + IMU

迁移自 isaac_arm_sim/shared/sensor_suite.py, 更新 import 路径和日志位置.

用法:
    suite = SensorSuite(calibration_dir, parent_prim_path)
    suite.build(world)
    world.reset()
    suite.initialize()
    ...
    data = suite.get_data()
"""

import os
import logging
import numpy as np
import yaml

# 文件日志
_log_path = os.path.join(os.path.dirname(__file__), "..", "..", "sensor_suite_debug.log")
_log_path = os.path.abspath(_log_path)
_log = logging.getLogger("sensor_suite")
_log.setLevel(logging.DEBUG)
_log.propagate = False
_fh = logging.FileHandler(_log_path, mode="w")
_fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
_log.addHandler(_fh)


def _load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _intrinsics_to_usd_camera_params(fx, fy, width, height):
    """
    从 pinhole 内参 (fx, fy, width, height) 反算 USD 相机的 focalLength 和 aperture.
    USD 相机约定:
        fx = focalLength / horizontalAperture * width
        fy = focalLength / verticalAperture * height
    固定 horizontalAperture = 20.955 mm (USD 默认值), 反算 focalLength.
    """
    h_aperture = 20.955  # mm, USD 默认
    focal_length = fx * h_aperture / width  # mm
    v_aperture = focal_length * height / fy  # mm
    return focal_length, h_aperture, v_aperture


def _set_or_create_attr(prim, attr_name, value, type_name):
    """设置或创建 USD prim 属性"""
    attr = prim.GetAttribute(attr_name)
    if not attr.IsValid():
        attr = prim.CreateAttribute(attr_name, type_name)
    attr.Set(value)


class SensorSuite:
    """
    传感器套件: 从 calibration 目录读取外参 + 内参文件,
    在指定的 parent prim 下创建所有传感器.

    支持两种 extrinsics.yaml 格式:
      1. 分层格式 (新): 顶层 ref_imu 定义套件安装变换 T_body_rimu,
         sensors 中各传感器 extrinsics 为 T_rimu_sensor.
         USD 层级: parent_prim / sensor_suite (Xform, T_body_rimu) / {rimu, camera1, ...}
      2. 扁平格式 (旧): 所有传感器直接挂在 parent_prim 下, extrinsics 相对于 body.
    """

    def __init__(self, calibration_dir: str, parent_prim_path: str,
                 sim_z_offset: float = 0.0):
        """
        Args:
            calibration_dir: calibration/ 目录的绝对路径
            parent_prim_path: 传感器挂载的父 prim 路径 (如 "/World/Quadruped/body")
            sim_z_offset: (仅旧格式) 仿真中所有传感器统一 z 偏移 (米).
                          新格式不需要此参数, 偏移已包含在 mount_extrinsics 中.
        """
        self._calibration_dir = calibration_dir
        self._parent_prim_path = parent_prim_path

        # 读取外参配置
        ext_path = os.path.join(calibration_dir, "extrinsics.yaml")
        self._ext_cfg = _load_yaml(ext_path)

        # 传感器实例字典
        self._sensors = {}
        # 传感器配置字典 (解析后)
        self._sensor_configs = {}

        # 检测格式: ref_imu 顶层键 → 新分层格式
        self._use_rimu = "ref_imu" in self._ext_cfg
        self._ref_imu_name = None       # rimu 的传感器名称 (e.g. "imu_center")
        self._suite_prim_path = None    # sensor_suite Xform 的 prim 路径

        if self._use_rimu:
            self._parse_rimu_format()
        else:
            self._parse_legacy_format(sim_z_offset)

    def _parse_rimu_format(self):
        """解析分层格式: ref_imu + sensors (T_rimu_sensor)"""
        rimu_cfg = self._ext_cfg["ref_imu"]
        self._ref_imu_name = rimu_cfg["name"]

        # T_body_rimu: 套件安装变换
        mount_ext = rimu_cfg.get("mount_extrinsics", {})
        self._mount_translation = np.array(
            mount_ext.get("translation", [0, 0, 0]), dtype=np.float64)
        self._mount_rotation = np.array(
            mount_ext.get("rotation", [1, 0, 0, 0]), dtype=np.float64)

        # ref_imu 自身: identity 变换 (它就是参考系原点)
        rimu_name = self._ref_imu_name
        self._sensor_configs[rimu_name] = {
            "type": "imu",
            "translation": np.zeros(3, dtype=np.float64),
            "rotation": np.array([1, 0, 0, 0], dtype=np.float64),
            "intrinsics": {},
            "raw_cfg": rimu_cfg,  # 含 frequency 等参数
        }

        # 其余传感器: T_rimu_sensor
        for name, cfg in self._ext_cfg.get("sensors", {}).items():
            sensor_type = cfg["type"]
            extrinsics = cfg.get("extrinsics", {})
            translation = np.array(
                extrinsics.get("translation", [0, 0, 0]), dtype=np.float64)
            rotation = np.array(
                extrinsics.get("rotation", [1, 0, 0, 0]), dtype=np.float64)

            intrinsics = self._load_intrinsics(cfg)

            self._sensor_configs[name] = {
                "type": sensor_type,
                "translation": translation,
                "rotation": rotation,  # wxyz
                "intrinsics": intrinsics,
                "raw_cfg": cfg,
            }

        _log.info("分层格式: ref_imu='%s', mount_t=%s, mount_r=%s, sensors=%s",
                  rimu_name, self._mount_translation.tolist(),
                  self._mount_rotation.tolist(),
                  [n for n in self._sensor_configs if n != rimu_name])

    def _parse_legacy_format(self, sim_z_offset: float):
        """解析旧扁平格式: 所有传感器相对于 body"""
        for name, cfg in self._ext_cfg.get("sensors", {}).items():
            sensor_type = cfg["type"]
            extrinsics = cfg.get("extrinsics", {})
            translation = np.array(
                extrinsics.get("translation", [0, 0, 0]), dtype=np.float64)
            translation[2] += sim_z_offset  # 仿真 z 偏移 (统一上移)
            rotation = np.array(
                extrinsics.get("rotation", [1, 0, 0, 0]), dtype=np.float64)

            intrinsics = self._load_intrinsics(cfg)

            self._sensor_configs[name] = {
                "type": sensor_type,
                "translation": translation,
                "rotation": rotation,  # wxyz
                "intrinsics": intrinsics,
                "raw_cfg": cfg,
            }

    def _load_intrinsics(self, cfg: dict) -> dict:
        """读取内参文件 (如果有)"""
        intrinsics = {}
        intrinsics_file = cfg.get("intrinsics_file")
        if intrinsics_file:
            int_path = os.path.join(self._calibration_dir, intrinsics_file)
            if os.path.isfile(int_path):
                intrinsics = _load_yaml(int_path)
            else:
                print(f"  警告: 内参文件不存在: {int_path}")
        return intrinsics

    @property
    def ref_imu_name(self) -> str | None:
        """返回 ref_imu 的传感器名称, 旧格式返回 None"""
        return self._ref_imu_name

    @property
    def _sensor_parent_path(self) -> str:
        """传感器 prim 的父路径: 新格式为 sensor_suite Xform, 旧格式为 parent_prim_path"""
        if self._use_rimu and self._suite_prim_path:
            return self._suite_prim_path
        return self._parent_prim_path

    def build(self, world):
        """
        根据配置在 world 中创建所有传感器 prim.
        必须在 world.reset() 之前调用.
        """
        print("  [SensorSuite] 开始创建传感器...")
        _log.info("=== build() 开始, configs: %s", list(self._sensor_configs.keys()))

        if self._use_rimu:
            self._build_rimu_xform()

        for name, cfg in self._sensor_configs.items():
            sensor_type = cfg["type"]
            _log.info("创建 sensor: name=%s, type=%s, translation=%s, rotation=%s",
                      name, sensor_type, cfg["translation"], cfg["rotation"])
            try:
                if sensor_type == "camera":
                    self._sensors[name] = self._create_camera(world, name, cfg)
                elif sensor_type == "lidar":
                    self._sensors[name] = self._create_lidar(world, name, cfg)
                elif sensor_type == "imu":
                    self._sensors[name] = self._create_imu(world, name, cfg)
                else:
                    print(f"  警告: 未知传感器类型 '{sensor_type}' (sensor: {name})")
                _log.info("  -> 创建成功: %s = %s", name, self._sensors.get(name))
            except Exception as e:
                _log.error("  -> 创建失败: %s: %s", name, e, exc_info=True)
                print(f"  错误: 创建 {name} 失败: {e}")
        print(f"  [SensorSuite] 共创建 {len(self._sensors)} 个传感器")
        _log.info("=== build() 完成, sensors: %s", list(self._sensors.keys()))

    def _build_rimu_xform(self):
        """创建 sensor_suite Xform prim, 承载 T_body_rimu 安装变换."""
        from pxr import UsdGeom, Gf
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        self._suite_prim_path = f"{self._parent_prim_path}/sensor_suite"
        xform = UsdGeom.Xform.Define(stage, self._suite_prim_path)

        # 设置 T_body_rimu 变换
        t = self._mount_translation
        q = self._mount_rotation  # wxyz
        xform.AddTranslateOp().Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
        xform.AddOrientOp().Set(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))

        _log.info("创建 sensor_suite Xform: %s, T_body_rimu: t=%s, q=%s",
                  self._suite_prim_path, t.tolist(), q.tolist())
        print(f"  [SensorSuite] Xform '{self._suite_prim_path}' "
              f"T_body_rimu: t={t.tolist()}, q={q.tolist()}")

    def initialize(self):
        """
        初始化所有传感器. 必须在 world.reset() 之后调用.
        """
        for name, sensor in self._sensors.items():
            sensor_type = self._sensor_configs[name]["type"]
            _log.info("initialize: %s (%s)", name, sensor_type)
            sensor.initialize()
            print(f"  [SensorSuite] 已初始化: {name} ({sensor_type})")

            if sensor_type == "camera":
                try:
                    sensor.add_distance_to_image_plane_to_frame()
                    _log.info("  -> 深度 annotator 注册成功")
                except Exception as e:
                    _log.error("  -> 深度 annotator 注册失败: %s", e, exc_info=True)

            if sensor_type == "lidar":
                # NOTE: transformPoints 已被官方弃用 (DEPRECATED), 属性存在但无实际效果.
                # 点云始终返回 sensor local frame.
                sensor.attach_annotator(
                    "IsaacCreateRTXLidarScanBuffer",
                    outputTimestamp=True,
                )

    def get_data(self) -> dict:
        """
        获取所有传感器的当前帧数据.

        Returns:
            dict: {sensor_name: data_dict}
                camera data_dict: {"rgb": ndarray, "depth": ndarray}
                lidar data_dict:  {"point_cloud": ndarray(N,3), "intensities": ndarray(N,)}
                imu data_dict:    {"lin_acc": ndarray(3,), "ang_vel": ndarray(3,), "orientation": ndarray(4,)}
        """
        result = {}
        for name, sensor in self._sensors.items():
            sensor_type = self._sensor_configs[name]["type"]
            try:
                if sensor_type == "camera":
                    result[name] = self._get_camera_data(sensor)
                elif sensor_type == "lidar":
                    result[name] = self._get_lidar_data(sensor)
                elif sensor_type == "imu":
                    result[name] = self._get_imu_data(sensor)
            except Exception as e:
                result[name] = {"error": str(e)}
        return result

    def get_data_by_type(self, sensor_type: str) -> dict:
        """
        只获取指定类型传感器的数据 (避免不必要的 GPU readback).

        Args:
            sensor_type: "camera", "lidar", 或 "imu"

        Returns:
            dict: {sensor_name: data_dict}
        """
        result = {}
        for name, sensor in self._sensors.items():
            if self._sensor_configs[name]["type"] != sensor_type:
                continue
            try:
                if sensor_type == "camera":
                    result[name] = self._get_camera_data(sensor)
                elif sensor_type == "lidar":
                    result[name] = self._get_lidar_data(sensor)
                elif sensor_type == "imu":
                    result[name] = self._get_imu_data(sensor)
            except Exception as e:
                result[name] = {"error": str(e)}
        return result

    def get_sensor(self, name: str):
        """获取指定名称的传感器实例"""
        return self._sensors.get(name)

    def get_sensor_names(self) -> list:
        """获取所有传感器名称"""
        return list(self._sensors.keys())

    def get_sensors_by_type(self, sensor_type: str) -> dict:
        """获取指定类型的所有传感器"""
        return {
            name: sensor
            for name, sensor in self._sensors.items()
            if self._sensor_configs[name]["type"] == sensor_type
        }

    def sync_transforms(self):
        """No-op: 保留接口兼容性."""
        pass

    # ---- 内部创建方法 ----

    def _create_camera(self, world, name: str, cfg: dict):
        """创建 Camera 传感器 (pinhole 模型)"""
        return self._create_camera_pinhole(world, name, cfg)

    def _create_camera_pinhole(self, world, name: str, cfg: dict):
        """创建 Pinhole 相机"""
        from isaacsim.sensors.camera import Camera

        intrinsics = cfg["intrinsics"]
        width = intrinsics.get("width", intrinsics.get("image_width", 640))
        height = intrinsics.get("height", intrinsics.get("image_height", 480))
        fx = intrinsics.get("fx", intrinsics.get("A11", 320.0))
        fy = intrinsics.get("fy", intrinsics.get("A22", 320.0))

        focal_length, h_aperture, v_aperture = _intrinsics_to_usd_camera_params(
            fx, fy, width, height
        )

        prim_path = f"{self._sensor_parent_path}/{name}"
        camera = Camera(
            prim_path=prim_path,
            name=name,
            resolution=(width, height),
            translation=cfg["translation"],
            orientation=cfg["rotation"],
        )

        camera.prim.GetAttribute("focalLength").Set(focal_length)
        camera.prim.GetAttribute("horizontalAperture").Set(h_aperture)
        camera.prim.GetAttribute("verticalAperture").Set(v_aperture)

        print(f"    Camera '{name}' [pinhole]: {width}x{height}, fl={focal_length:.2f}mm, "
              f"pos={cfg['translation'].tolist()}")
        return camera

    def _create_lidar(self, world, name: str, cfg: dict):
        """创建并配置一个 RTX LiDAR 传感器.

        支持三种 config 来源:
          1. 内置配置名: config_name: "ZVISION_ML30S"
          2. 自定义 JSON/USD 文件: config_name: "lidar_configs/My_Lidar.json"
          3. 内联参数自动生成: config_name: "auto" (暂未迁移)
        """
        from isaacsim.sensors.rtx import LidarRtx

        intrinsics = cfg["intrinsics"]
        model = intrinsics.get("model", "rtx_lidar")

        if model == "rtx_lidar":
            config_name = intrinsics.get("config_name", "Example_Rotary")
        else:
            print(f"    警告: LiDAR model '{model}' 暂不支持, 回退到 Example_Rotary")
            config_name = "Example_Rotary"

        prim_path = f"{self._sensor_parent_path}/{name}"

        # 自定义 JSON/USD 文件 → 通过 USD reference 加载
        if config_name.endswith(".json"):
            usd_path = self._resolve_custom_lidar_usd(config_name)
            if usd_path:
                lidar = self._create_lidar_from_usd(
                    usd_path, prim_path, name, cfg)
                print(f"    LiDAR '{name}': custom USD={usd_path}, "
                      f"pos={cfg['translation'].tolist()}")
            else:
                print(f"    警告: 找不到 USD, 回退到 Example_Rotary")
                lidar = LidarRtx(
                    prim_path=prim_path, name=name,
                    translation=cfg["translation"],
                    orientation=cfg["rotation"],
                    config_file_name="Example_Rotary",
                )
        else:
            # 内置配置名 → 走官方 LidarRtx 流程
            lidar = LidarRtx(
                prim_path=prim_path, name=name,
                translation=cfg["translation"],
                orientation=cfg["rotation"],
                config_file_name=config_name,
            )
            print(f"    LiDAR '{name}': config={config_name}, "
                  f"pos={cfg['translation'].tolist()}")

        # attr_overrides
        attr_overrides = intrinsics.get("attr_overrides")
        if attr_overrides:
            self._apply_attr_overrides(prim_path, attr_overrides)

        self._dump_lidar_prim_attrs(prim_path)

        return lidar

    def _resolve_custom_lidar_usd(self, json_path: str) -> str | None:
        """解析自定义 JSON 路径, 查找对应的 .usd 文件."""
        print(f"    [LiDAR Debug] 输入 config_name: {json_path}")
        print(f"    [LiDAR Debug] calibration_dir: {self._calibration_dir}")
        if not os.path.isabs(json_path):
            # calibration_dir is .../configs/calibration/pinhole/, need to go up 2 levels to reach configs/
            calibration_parent = os.path.dirname(self._calibration_dir)  # .../calibration
            configs_dir = os.path.dirname(calibration_parent)              # .../configs
            project_root = os.path.dirname(configs_dir)                     # .../r2s2r
            workspace_root = os.path.dirname(project_root)                # .../isaac_real2sim2real_go2
            print(f"    [LiDAR Debug] 搜索目录: configs_dir={configs_dir}")
            print(f"    [LiDAR Debug] 搜索目录: project_root={project_root}")
            print(f"    [LiDAR Debug] 搜索目录: workspace_root={workspace_root}")
            for root in [configs_dir, project_root, workspace_root]:
                candidate = os.path.join(root, json_path)
                print(f"    [LiDAR Debug] 检查: {candidate} -> 存在: {os.path.isfile(candidate)}")
                if os.path.isfile(candidate):
                    json_path = candidate
                    print(f"    [LiDAR Debug] 找到 JSON: {json_path}")
                    break
            else:
                json_path = os.path.join(configs_dir, json_path)
                print(f"    [LiDAR Debug] 未找到，默认使用: {json_path}")
        json_path = os.path.abspath(json_path)
        print(f"    [LiDAR Debug] 最终 JSON 绝对路径: {json_path}")

        stem = os.path.splitext(os.path.basename(json_path))[0]
        json_dir = os.path.dirname(json_path)

        candidates = [
            os.path.join(json_dir, f"{stem}.usd"),
            os.path.join(json_dir, f"{stem}.usda"),
            os.path.join(os.path.dirname(json_dir), "usd", f"{stem}.usd"),
            os.path.join(os.path.dirname(json_dir), "usd", f"{stem}.usda"),
        ]
        for p in candidates:
            if os.path.isfile(p):
                print(f"    自定义 LiDAR USD: {p}")
                return p

        print(f"    警告: 找不到对应的 USD 文件, 已搜索: {candidates}")
        return None

    @staticmethod
    def _create_lidar_from_usd(
        usd_path: str, prim_path: str, name: str, cfg: dict,
    ):
        """从自定义 .usd 文件创建 OmniLidar prim 并包装为 LidarRtx."""
        from isaacsim.sensors.rtx import LidarRtx
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.xforms import reset_and_set_xform_ops
        from pxr import Gf
        import omni.usd

        add_reference_to_stage(
            usd_path=usd_path, prim_path=prim_path, prim_type="OmniLidar")

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        t = cfg["translation"]
        q = cfg["rotation"]
        reset_and_set_xform_ops(
            prim, Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])),
            Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))

        attr = prim.GetAttribute("omni:sensor:Core:skipDroppingInvalidPoints")
        if attr.IsValid():
            attr.Set(True)
            print(f"    [USD] skipDroppingInvalidPoints=True")

        lidar = LidarRtx(prim_path=prim_path, name=name)
        return lidar

    @staticmethod
    def _apply_attr_overrides(prim_path: str, overrides: dict):
        """对 omni:sensor:Core:* 标量属性做最小修改."""
        import omni.usd
        from pxr import Usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"    [attr_overrides] prim {prim_path} 无效, 跳过")
            return

        if prim.GetTypeName() != "OmniLidar":
            for child in Usd.PrimRange(prim):
                if child.GetTypeName() == "OmniLidar":
                    prim = child
                    break

        for key, value in overrides.items():
            attr_name = f"omni:sensor:Core:{key}"
            attr = prim.GetAttribute(attr_name)
            if attr.IsValid():
                old_val = attr.Get()
                attr.Set(value)
                new_val = attr.Get()
                print(f"    [attr_overrides] {attr_name}: {old_val} → {new_val}")
            else:
                print(f"    [attr_overrides] {attr_name} 不存在, 跳过")

    @staticmethod
    def _dump_lidar_prim_attrs(prim_path: str):
        """诊断: 打印 OmniLidar prim 的关键属性."""
        from pxr import Usd
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"    [LiDAR Diag] prim {prim_path} 无效")
            return

        target = prim
        if prim.GetTypeName() != "OmniLidar":
            for child in Usd.PrimRange(prim):
                if child.GetTypeName() == "OmniLidar":
                    target = child
                    break

        print(f"    [LiDAR Diag] prim={target.GetPath()}, type={target.GetTypeName()}")

        keywords = ["sensor", "model", "config", "return", "emitter",
                     "maxReturn", "profile", "scan", "lidar", "Lidar"]
        for attr in target.GetAttributes():
            name = attr.GetName()
            if any(kw.lower() in name.lower() for kw in keywords):
                try:
                    val = attr.Get()
                    val_str = str(val)
                    if len(val_str) > 120:
                        val_str = val_str[:120] + "..."
                    print(f"      {name} = {val_str}")
                except Exception:
                    print(f"      {name} = <error reading>")

    def _create_imu(self, world, name: str, cfg: dict):
        """创建 IMU 传感器"""
        from isaacsim.sensors.physics import IMUSensor

        raw_cfg = cfg["raw_cfg"]
        frequency = raw_cfg.get("frequency", 200)

        prim_path = f"{self._sensor_parent_path}/{name}"
        imu = IMUSensor(
            prim_path=prim_path,
            name=name,
            frequency=frequency,
            translation=cfg["translation"],
            orientation=cfg["rotation"],
        )

        print(f"    IMU '{name}': freq={frequency}Hz, pos={cfg['translation'].tolist()}")
        return imu

    # ---- 内部数据获取方法 ----

    @staticmethod
    def _get_camera_data(camera) -> dict:
        """获取 Camera 的 RGB 和 Depth 数据"""
        data = {}
        rgb = camera.get_rgb()
        if rgb is not None and rgb.size > 0:
            data["rgb"] = rgb

        cf = camera.get_current_frame()
        depth_raw = cf.get("distance_to_image_plane")
        if depth_raw is not None:
            data["depth"] = np.squeeze(depth_raw).astype(np.float32)

        return data

    @staticmethod
    def _get_lidar_data(lidar) -> dict:
        """获取 LiDAR 的点云数据"""
        data = {}
        cf = lidar.get_current_frame()
        scan_buf = cf.get("IsaacCreateRTXLidarScanBuffer")
        if scan_buf is not None:
            xyz = scan_buf.get("data")
            if xyz is not None and hasattr(xyz, '__len__') and len(xyz) > 0:
                pc = np.asarray(xyz, dtype=np.float32)
                if pc.ndim == 2 and pc.shape[1] >= 3:
                    data["point_cloud"] = pc[:, :3]
                elif pc.ndim == 1 and len(pc) >= 3:
                    data["point_cloud"] = pc.reshape(-1, 3)
            intensity = scan_buf.get("intensity")
            if intensity is not None and hasattr(intensity, '__len__') and len(intensity) > 0:
                data["intensities"] = np.asarray(intensity, dtype=np.float32)
            fire_times = scan_buf.get("timestamp")
            if fire_times is not None and hasattr(fire_times, '__len__') and len(fire_times) > 0:
                data["fire_times_ns"] = np.asarray(fire_times, dtype=np.float64)
            info = scan_buf.get("info")
            if info is not None:
                data["scan_info"] = info
        return data

    @staticmethod
    def _get_imu_data(imu) -> dict:
        """获取 IMU 的加速度、角速度、姿态数据"""
        cf = imu.get_current_frame()
        data = {}
        lin_acc = cf.get("lin_acc")
        if lin_acc is not None:
            data["lin_acc"] = np.array(lin_acc, dtype=np.float32).flatten()
        ang_vel = cf.get("ang_vel")
        if ang_vel is not None:
            data["ang_vel"] = np.array(ang_vel, dtype=np.float32).flatten()
        orientation = cf.get("orientation")
        if orientation is not None:
            data["orientation"] = np.array(orientation, dtype=np.float32).flatten()
        return data
