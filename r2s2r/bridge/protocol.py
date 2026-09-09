"""
protocol.py
TCP 桥接消息协议 — Isaac Sim ↔ ROS1 Docker

消息格式:
    [4B magic: 0x49534252 "ISBR"] [4B msg_type] [4B payload_len] [payload: msgpack bytes]

此文件需与 ros1_ws/src/sim_bridge/src/protocol.py 保持同步.
"""

import struct
import time
import numpy as np

try:
    import msgpack
except ImportError:
    msgpack = None
    print("[protocol] WARNING: msgpack not installed. pip install msgpack")

# ---- 常量 ----

MAGIC = 0x49534252  # "ISBR" in ASCII
HEADER_SIZE = 12    # magic(4) + msg_type(4) + payload_len(4)
HEADER_FMT = "<III"  # little-endian: uint32 * 3

# 消息类型
MSG_POSE = 0x01        # Isaac → ROS1: 机器人位姿 + 速度
MSG_POINTCLOUD = 0x02  # Isaac → ROS1: LiDAR 点云
MSG_CMD_VEL = 0x03     # ROS1 → Isaac: 速度指令
MSG_IMAGE = 0x04       # Isaac → ROS1: Camera 图像 (JPEG 压缩)
MSG_IMU = 0x05         # Isaac → ROS1: IMU 数据
MSG_RESET = 0x30       # ROS1 → Isaac: 复位请求 (to_start 或 原地恢复)

MSG_TYPE_NAMES = {
    MSG_POSE: "POSE",
    MSG_POINTCLOUD: "POINTCLOUD",
    MSG_CMD_VEL: "CMD_VEL",
    MSG_IMAGE: "IMAGE",
    MSG_IMU: "IMU",
    MSG_RESET: "RESET",
}


# ---- 编码 ----

def encode_message(msg_type: int, payload: dict) -> bytes:
    """将消息编码为 bytes (header + msgpack payload)"""
    if msgpack is None:
        raise RuntimeError("msgpack not installed")
    payload_bytes = msgpack.packb(payload, use_bin_type=True)
    header = struct.pack(HEADER_FMT, MAGIC, msg_type, len(payload_bytes))
    return header + payload_bytes


def encode_pose(x, y, z, qx, qy, qz, qw,
                vx=0.0, vy=0.0, vz=0.0,
                wx=0.0, wy=0.0, wz=0.0,
                timestamp=None) -> bytes:
    """编码 POSE 消息"""
    if timestamp is None:
        timestamp = time.time()
    payload = {
        "ts": timestamp,
        "x": float(x), "y": float(y), "z": float(z),
        "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw),
        "vx": float(vx), "vy": float(vy), "vz": float(vz),
        "wx": float(wx), "wy": float(wy), "wz": float(wz),
    }
    return encode_message(MSG_POSE, payload)


def encode_pointcloud(xyz: np.ndarray, frame_id: str = "sensor",
                      timestamp=None) -> bytes:
    """编码 POINTCLOUD 消息
    Args:
        xyz: ndarray (N, 3), float32 点云
        frame_id: 坐标系名称
        timestamp: 时间戳 (默认 time.time())
    """
    if timestamp is None:
        timestamp = time.time()
    xyz_f32 = np.ascontiguousarray(xyz[:, :3], dtype=np.float32)
    payload = {
        "ts": timestamp,
        "frame_id": frame_id,
        "n_points": int(xyz_f32.shape[0]),
        "xyz": xyz_f32.tobytes(),
    }
    return encode_message(MSG_POINTCLOUD, payload)


def encode_cmd_vel(vx: float, vy: float, wz: float) -> bytes:
    """编码 CMD_VEL 消息"""
    payload = {"vx": float(vx), "vy": float(vy), "wz": float(wz)}
    return encode_message(MSG_CMD_VEL, payload)


def encode_reset(to_start: bool = True) -> bytes:
    """编码 RESET 消息
    Args:
        to_start: True = 回 config 起点 (yaw=0); False = 原地恢复 (保留 xy/yaw, 清 roll/pitch)
    """
    payload = {"to_start": bool(to_start)}
    return encode_message(MSG_RESET, payload)


def encode_image(rgb: np.ndarray, frame_id: str = "camera",
                 quality: int = 80, timestamp=None) -> bytes:
    """编码 IMAGE 消息 (JPEG 压缩)
    Args:
        rgb: ndarray (H, W, 3), uint8 RGB 图像
        frame_id: 坐标系名称
        quality: JPEG 压缩质量 (1-100)
        timestamp: 时间戳
    """
    import cv2
    if timestamp is None:
        timestamp = time.time()
    # BGR for cv2
    bgr = rgb[:, :, ::-1] if rgb.shape[2] == 3 else rgb
    _, jpeg_buf = cv2.imencode('.jpg', bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, quality])
    payload = {
        "ts": timestamp,
        "frame_id": frame_id,
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
        "encoding": "jpeg",
        "data": jpeg_buf.tobytes(),
    }
    return encode_message(MSG_IMAGE, payload)


def encode_imu(lin_acc: np.ndarray, ang_vel: np.ndarray,
               orientation: np.ndarray = None,
               frame_id: str = "imu",
               timestamp=None) -> bytes:
    """编码 IMU 消息
    Args:
        lin_acc: ndarray (3,) 线加速度 [ax, ay, az]
        ang_vel: ndarray (3,) 角速度 [wx, wy, wz]
        orientation: ndarray (4,) 四元数 [qx, qy, qz, qw] (可选)
        frame_id: 坐标系名称
        timestamp: 时间戳
    """
    if timestamp is None:
        timestamp = time.time()
    payload = {
        "ts": timestamp,
        "frame_id": frame_id,
        "ax": float(lin_acc[0]), "ay": float(lin_acc[1]), "az": float(lin_acc[2]),
        "wx": float(ang_vel[0]), "wy": float(ang_vel[1]), "wz": float(ang_vel[2]),
    }
    if orientation is not None and len(orientation) >= 4:
        payload["qx"] = float(orientation[0])
        payload["qy"] = float(orientation[1])
        payload["qz"] = float(orientation[2])
        payload["qw"] = float(orientation[3])
    return encode_message(MSG_IMU, payload)


# ---- 解码 ----

def decode_header(data: bytes) -> tuple:
    """解码消息头 (12 bytes)
    Returns:
        (magic, msg_type, payload_len) or None if invalid
    """
    if len(data) < HEADER_SIZE:
        return None
    magic, msg_type, payload_len = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    if magic != MAGIC:
        return None
    return magic, msg_type, payload_len


def decode_payload(msg_type: int, payload_bytes: bytes) -> dict:
    """解码 msgpack payload"""
    if msgpack is None:
        raise RuntimeError("msgpack not installed")
    data = msgpack.unpackb(payload_bytes, raw=False)

    # POINTCLOUD: 还原 xyz bytes → ndarray
    if msg_type == MSG_POINTCLOUD and "xyz" in data:
        n = data["n_points"]
        data["xyz"] = np.frombuffer(data["xyz"], dtype=np.float32).reshape(n, 3)

    return data


def decode_message(buffer: bytes) -> tuple:
    """从 buffer 解码一条完整消息
    Returns:
        (msg_type, payload_dict, consumed_bytes) or (None, None, 0) if incomplete
    """
    hdr = decode_header(buffer)
    if hdr is None:
        return None, None, 0
    _, msg_type, payload_len = hdr
    total = HEADER_SIZE + payload_len
    if len(buffer) < total:
        return None, None, 0
    payload = decode_payload(msg_type, buffer[HEADER_SIZE:total])
    return msg_type, payload, total
