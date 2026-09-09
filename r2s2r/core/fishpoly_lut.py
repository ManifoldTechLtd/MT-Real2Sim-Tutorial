"""
FishPoly 鱼眼相机 → OmniLensDistortionLutAPI EXR 纹理生成

FishPoly 模型:
    r(θ) = θ + k2·θ² + k3·θ³ + k4·θ⁴ + k5·θ⁵ + k6·θ⁶ + k7·θ⁷
    u = A11·r·cosφ + A12·r·sinφ + u0
    v = A22·r·sinφ + v0

生成两张 EXR 纹理 (用于 Isaac Sim OmniLensDistortionLutAPI):
    1. rayEnterDirectionTexture — 反投影 (NDC → ray direction), 3 通道 RGB=XYZ
    2. rayExitPositionTexture  — 正投影 (ray direction → NDC), 2 通道 RG=UV

参考: https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html
"""

import os
import hashlib
import json
import numpy as np


def fishpoly_r(theta, coeffs):
    """FishPoly 正向投影: θ → r"""
    # r(θ) = θ + k2·θ² + k3·θ³ + ... + k7·θ⁷
    k2, k3, k4, k5, k6, k7 = coeffs
    t2 = theta * theta
    t3 = t2 * theta
    t4 = t3 * theta
    t5 = t4 * theta
    t6 = t5 * theta
    t7 = t6 * theta
    return theta + k2 * t2 + k3 * t3 + k4 * t4 + k5 * t5 + k6 * t6 + k7 * t7


def fishpoly_dr(theta, coeffs):
    """FishPoly 正向投影的导数: dr/dθ (用于 Newton 迭代)"""
    k2, k3, k4, k5, k6, k7 = coeffs
    t = theta
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    t6 = t5 * t
    return 1.0 + 2*k2*t + 3*k3*t2 + 4*k4*t3 + 5*k5*t4 + 6*k6*t5 + 7*k7*t6


def fishpoly_inverse_r(r_target, coeffs, max_theta=None, tol=1e-10, max_iter=50):
    """
    FishPoly 反投影: r → θ (Newton-Raphson 向量化)

    Args:
        r_target: ndarray, 目标 r 值
        coeffs: (k2, k3, k4, k5, k6, k7)
        max_theta: 最大 θ (弧度), 默认 2π/3 (120°)
        tol: 收敛容差
        max_iter: 最大迭代次数

    Returns:
        theta: ndarray, 对应的 θ 值
    """
    if max_theta is None:
        max_theta = np.deg2rad(120)

    # 初始猜测: θ ≈ r (小角度近似)
    theta = np.clip(r_target.copy(), 0, max_theta)

    for _ in range(max_iter):
        r_est = fishpoly_r(theta, coeffs)
        dr = fishpoly_dr(theta, coeffs)
        # 避免除零
        dr = np.where(np.abs(dr) < 1e-15, 1e-15, dr)
        delta = (r_est - r_target) / dr
        theta = theta - delta
        theta = np.clip(theta, 0, max_theta)
        if np.max(np.abs(delta)) < tol:
            break

    return theta


def _oct_to_unit_vector(x, y):
    """Octahedral decoding: UV [-1,1]² → 单位方向向量"""
    dirX, dirY = np.meshgrid(x, y)
    dirZ = 1.0 - np.abs(dirX) - np.abs(dirY)

    sx = 2.0 * np.heaviside(dirX, 1.0) - 1.0
    sy = 2.0 * np.heaviside(dirY, 1.0) - 1.0

    tmpX = dirX.copy()
    dirX = np.where(dirZ <= 0, (1.0 - np.abs(dirY)) * sx, dirX)
    dirY = np.where(dirZ <= 0, (1.0 - np.abs(tmpX)) * sy, dirY)

    n = 1.0 / np.sqrt(dirX**2 + dirY**2 + dirZ**2)
    return dirX * n, dirY * n, dirZ * n


def _unit_vector_to_oct(dirX, dirY, dirZ):
    """Octahedral encoding: 单位方向向量 → UV [-1,1]²"""
    n = 1.0 / (np.abs(dirX) + np.abs(dirY) + np.abs(dirZ))
    octX = dirX * n
    octY = dirY * n

    sx = 2.0 * np.heaviside(octX, 1.0) - 1.0
    sy = 2.0 * np.heaviside(octY, 1.0) - 1.0

    tmpX = octX.copy()
    octX = np.where(dirZ <= 0, (1.0 - np.abs(octY)) * sx, octX)
    octY = np.where(dirZ <= 0, (1.0 - np.abs(tmpX)) * sy, octY)
    return octX, octY


def _fishpoly_project(dirX, dirY, dirZ, params):
    """
    FishPoly 正向投影: 3D 方向 → NDC [0,1]²

    Args:
        dirX, dirY, dirZ: 方向向量分量 (ndarray)
        params: dict with keys: A11, A12, A22, u0, v0, coeffs, width, height

    Returns:
        ndc_x, ndc_y: NDC 坐标 [0,1]
    """
    coeffs = params["coeffs"]
    A11 = params["A11"]
    A12 = params["A12"]
    A22 = params["A22"]
    u0 = params["u0"]
    v0 = params["v0"]
    W = params["width"]
    H = params["height"]

    # RTX LUT 约定: view space 坐标系 X右 Y上 Z前
    # 图像坐标系: u右 v下
    # 需要翻转 Y: view_Y_up → image_v_down
    # θ = 入射角 = angle between ray and +Z axis
    sin_theta = np.sqrt(dirX**2 + dirY**2)
    theta = np.arctan2(sin_theta, dirZ)

    # 方位角 (注意 -dirY: view space Y向上 → 图像 v向下)
    phi = np.arctan2(-dirY, dirX)

    # FishPoly 投影: θ → r
    r = fishpoly_r(theta, coeffs)

    # 像素坐标
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    u = A11 * r * cos_phi + A12 * r * sin_phi + u0
    v = A22 * r * sin_phi + v0

    # 像素 → NDC [0, 1]
    ndc_x = u / W
    ndc_y = v / H

    # 超出 maxIncidentAngle 的标记为无效 (NDC 中心)
    max_theta = params.get("max_theta", np.deg2rad(120))
    invalid = theta > max_theta
    ndc_x = np.where(invalid, 0.5, ndc_x)
    ndc_y = np.where(invalid, 0.5, ndc_y)

    return ndc_x, ndc_y


def _fishpoly_unproject(ndc_x_arr, ndc_y_arr, params):
    """
    FishPoly 反投影: NDC [0,1]² → 3D 方向

    Args:
        ndc_x_arr, ndc_y_arr: 1D arrays, NDC 坐标
        params: dict

    Returns:
        dirX, dirY, dirZ: 方向向量分量 (2D ndarray, shape: [H_tex, W_tex])
    """
    coeffs = params["coeffs"]
    A11 = params["A11"]
    A12 = params["A12"]
    A22 = params["A22"]
    u0 = params["u0"]
    v0 = params["v0"]
    W = params["width"]
    H = params["height"]

    # NDC → 像素坐标
    U, V = np.meshgrid(ndc_x_arr * W, ndc_y_arr * H)

    # 像素 → 归一化 (反仿射变换)
    # u - u0 = A11 * r * cosφ + A12 * r * sinφ
    # v - v0 = A22 * r * sinφ
    du = U - u0
    dv = V - v0

    # 求解 r*sinφ 和 r*cosφ
    r_sin_phi = dv / A22
    r_cos_phi = (du - A12 * r_sin_phi) / A11

    # r 和 φ
    r = np.sqrt(r_cos_phi**2 + r_sin_phi**2)
    phi = np.arctan2(r_sin_phi, r_cos_phi)

    # r → θ (Newton 反求)
    max_theta = params.get("max_theta", np.deg2rad(120))
    theta = fishpoly_inverse_r(r, coeffs, max_theta=max_theta)

    # θ, φ → 3D 方向 (RTX LUT 约定: X右 Y上 Z前)
    # 注意: phi 是图像坐标系的方位角 (v向下), 需要翻转 Y
    sin_theta = np.sin(theta)
    dirX = sin_theta * np.cos(phi)
    dirY = -sin_theta * np.sin(phi)  # 取反: 图像v向下 → view space Y向上
    dirZ = np.cos(theta)

    # 超出范围的标记为无效 (指向光轴 +Z)
    invalid = (theta >= max_theta) | (r < 1e-10)
    dirX = np.where(invalid & (r < 1e-10), 0.0, dirX)
    dirY = np.where(invalid & (r < 1e-10), 0.0, dirY)
    dirZ = np.where(invalid & (r < 1e-10), 1.0, dirZ)  # 光轴方向
    dirX = np.where(invalid & (r >= 1e-10), 0.0, dirX)
    dirY = np.where(invalid & (r >= 1e-10), 0.0, dirY)
    dirZ = np.where(invalid & (r >= 1e-10), -1.0, dirZ)  # 无效方向 (背向)

    return dirX, dirY, dirZ


def generate_fishpoly_lut(intrinsics: dict, cache_dir: str,
                          tex_width: int = None, tex_height: int = None) -> tuple:
    """
    生成 FishPoly LUT 纹理 (EXR)。如果缓存有效则直接返回路径。

    Args:
        intrinsics: FishPoly 内参字典 (从 YAML 加载)
        cache_dir: 缓存目录
        tex_width: 纹理宽度 (默认 = 图像宽度)
        tex_height: 纹理高度 (默认 = 图像高度)

    Returns:
        (direction_exr_path, ndc_exr_path): 两张 EXR 纹理的路径
    """
    W = intrinsics["image_width"]
    H = intrinsics["image_height"]
    if tex_width is None:
        tex_width = W
    if tex_height is None:
        tex_height = H

    # 构建参数字典
    params = {
        "coeffs": (
            intrinsics["k2"], intrinsics["k3"], intrinsics["k4"],
            intrinsics["k5"], intrinsics["k6"], intrinsics["k7"],
        ),
        "A11": intrinsics["A11"],
        "A12": intrinsics.get("A12", 0.0),
        "A22": intrinsics["A22"],
        "u0": intrinsics["u0"],
        "v0": intrinsics["v0"],
        "width": W,
        "height": H,
        "max_theta": np.deg2rad(intrinsics.get("maxIncidentAngle", 120)),
    }

    # 缓存检查: 参数 hash
    param_str = json.dumps({k: (list(v) if isinstance(v, tuple) else v)
                            for k, v in params.items()}, sort_keys=True)
    param_hash = hashlib.md5(f"{param_str}_{tex_width}x{tex_height}".encode()).hexdigest()[:12]

    os.makedirs(cache_dir, exist_ok=True)
    direction_path = os.path.join(cache_dir, f"fishpoly_direction_{param_hash}.exr")
    ndc_path = os.path.join(cache_dir, f"fishpoly_ndc_{param_hash}.exr")
    meta_path = os.path.join(cache_dir, f"fishpoly_meta_{param_hash}.json")

    if os.path.isfile(direction_path) and os.path.isfile(ndc_path) and os.path.isfile(meta_path):
        print(f"  [FishPoly LUT] 使用缓存: {cache_dir}")
        return direction_path, ndc_path

    print(f"  [FishPoly LUT] 生成 {tex_width}x{tex_height} EXR 纹理...")

    # ---- 反投影纹理 (NDC → direction) ----
    ndc_x = np.linspace(0, 1, tex_width, dtype=np.float64)
    ndc_y = np.linspace(0, 1, tex_height, dtype=np.float64)

    dirX, dirY, dirZ = _fishpoly_unproject(ndc_x, ndc_y, params)

    direction_img = np.zeros((tex_height, tex_width, 3), dtype=np.float32)
    direction_img[:, :, 0] = dirX.astype(np.float32)
    direction_img[:, :, 1] = dirY.astype(np.float32)
    direction_img[:, :, 2] = dirZ.astype(np.float32)

    # ---- 正投影纹理 (direction → NDC, octahedral encoding) ----
    oct_x = np.linspace(-1, 1, tex_width, dtype=np.float64)
    oct_y = np.linspace(-1, 1, tex_height, dtype=np.float64)
    oct_dirX, oct_dirY, oct_dirZ = _oct_to_unit_vector(oct_x, oct_y)

    proj_ndc_x, proj_ndc_y = _fishpoly_project(oct_dirX, oct_dirY, oct_dirZ, params)

    ndc_img = np.zeros((tex_height, tex_width, 2), dtype=np.float32)
    ndc_img[:, :, 0] = proj_ndc_x.astype(np.float32)
    ndc_img[:, :, 1] = proj_ndc_y.astype(np.float32)

    # ---- 保存 EXR ----
    _save_exr(direction_path, direction_img)
    _save_exr(ndc_path, ndc_img)

    # 保存元信息
    with open(meta_path, "w") as f:
        json.dump({
            "tex_width": tex_width,
            "tex_height": tex_height,
            "image_width": W,
            "image_height": H,
            "param_hash": param_hash,
        }, f, indent=2)

    print(f"  [FishPoly LUT] 已生成: {direction_path}")
    print(f"  [FishPoly LUT] 已生成: {ndc_path}")
    return direction_path, ndc_path


def _save_exr(path: str, img_data: np.ndarray):
    """保存 float32 ndarray 为 EXR 文件 (无外部依赖, 纯 Python 实现)。

    写入未压缩 scanline 格式的 OpenEXR 文件。
    参考: https://openexr.com/en/latest/OpenEXRFileLayout.html

    Args:
        path: 输出路径
        img_data: shape (H, W, C), C=2 或 3, dtype=float32
    """
    import struct

    height, width, channels = img_data.shape
    channel_names = ['B', 'G', 'R'][:channels]  # EXR 通道按字母序存储
    # 重排数据: img_data 的通道顺序是 R,G,B -> EXR 存储顺序 B,G,R
    if channels == 3:
        exr_data = img_data[:, :, ::-1]  # RGB -> BGR
    elif channels == 2:
        channel_names = ['G', 'R']  # 2通道: 按字母序 G, R
        exr_data = img_data[:, :, ::-1]  # RG -> GR
    else:
        channel_names = ['R']
        exr_data = img_data

    # ---- Header ----
    header = bytearray()

    def _write_attr(buf, name, type_name, value_bytes):
        buf += name.encode('ascii') + b'\x00'
        buf += type_name.encode('ascii') + b'\x00'
        buf += struct.pack('<I', len(value_bytes))
        buf += value_bytes

    # channels
    ch_bytes = bytearray()
    for ch_name in channel_names:
        ch_bytes += ch_name.encode('ascii') + b'\x00'
        # pixel type: 2 = FLOAT
        ch_bytes += struct.pack('<I', 2)
        # pLinear, reserved[3], xSampling, ySampling
        ch_bytes += struct.pack('<B3xII', 0, 1, 1)
    ch_bytes += b'\x00'  # end of channel list
    _write_attr(header, 'channels', 'chlist', bytes(ch_bytes))

    # compression: 0 = NO_COMPRESSION
    _write_attr(header, 'compression', 'compression', struct.pack('<B', 0))

    # dataWindow
    _write_attr(header, 'dataWindow', 'box2i',
                struct.pack('<iiii', 0, 0, width - 1, height - 1))

    # displayWindow
    _write_attr(header, 'displayWindow', 'box2i',
                struct.pack('<iiii', 0, 0, width - 1, height - 1))

    # lineOrder: 0 = INCREASING_Y
    _write_attr(header, 'lineOrder', 'lineOrder', struct.pack('<B', 0))

    # pixelAspectRatio
    _write_attr(header, 'pixelAspectRatio', 'float', struct.pack('<f', 1.0))

    # screenWindowCenter
    _write_attr(header, 'screenWindowCenter', 'v2f', struct.pack('<ff', 0.0, 0.0))

    # screenWindowWidth
    _write_attr(header, 'screenWindowWidth', 'float', struct.pack('<f', 1.0))

    header += b'\x00'  # end of header

    # ---- Compute offsets ----
    magic = struct.pack('<I', 0x01312F76)
    version = struct.pack('<I', 2)  # version 2, single-part scanline

    header_size = len(magic) + len(version) + len(header)
    offset_table_size = height * 8  # uint64 per scanline

    scanline_pixel_size = width * channels * 4  # float32 = 4 bytes
    scanline_record_size = 4 + 4 + scanline_pixel_size  # y_coord + data_size + pixels

    offsets = []
    current_offset = header_size + offset_table_size
    for y in range(height):
        offsets.append(current_offset)
        current_offset += scanline_record_size

    # ---- Write file ----
    with open(path, 'wb') as f:
        f.write(magic)
        f.write(version)
        f.write(header)

        # Offset table
        for off in offsets:
            f.write(struct.pack('<Q', off))

        # Scanline data
        exr_flat = exr_data.astype(np.float32)
        for y in range(height):
            f.write(struct.pack('<i', y))  # y coordinate
            f.write(struct.pack('<I', scanline_pixel_size))  # data size
            # 通道交织: 先写所有像素的 ch0, 再写 ch1, ...
            for c in range(channels):
                f.write(exr_flat[y, :, c].tobytes())



# ---- CLI 测试入口 ----
if __name__ == "__main__":
    import yaml
    import sys

    if len(sys.argv) < 2:
        print("用法: python fishpoly_lut.py <intrinsics_yaml> [cache_dir]")
        sys.exit(1)

    yaml_path = sys.argv[1]
    cache_dir = sys.argv[2] if len(sys.argv) > 2 else "./lut_cache"

    with open(yaml_path) as f:
        intrinsics = yaml.safe_load(f)

    dir_path, ndc_path = generate_fishpoly_lut(intrinsics, cache_dir)
    print(f"\nDirection texture: {dir_path}")
    print(f"NDC texture:       {ndc_path}")
