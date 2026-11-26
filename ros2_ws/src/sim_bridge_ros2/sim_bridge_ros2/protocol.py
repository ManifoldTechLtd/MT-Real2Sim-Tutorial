"""
protocol.py (ROS1 侧)
TCP 桥接消息协议 — 与 isaacsim/bridge/protocol.py 保持同步

此文件是 isaacsim/bridge/protocol.py 的精简副本, 仅含解码/编码逻辑.
"""

import struct
import numpy as np

try:
    import msgpack
except ImportError:
    msgpack = None
    print("[protocol] WARNING: msgpack not installed. pip3 install msgpack")

# ---- 常量 ----

MAGIC = 0x49534252  # "ISBR"
HEADER_SIZE = 12
HEADER_FMT = "<III"

MSG_POSE = 0x01
MSG_POINTCLOUD = 0x02
MSG_CMD_VEL = 0x03
MSG_IMAGE = 0x04
MSG_IMU = 0x05
MSG_CLOCK = 0x06
MSG_RESET = 0x30
MSG_SET_POSE = 0x31    # ROS → Isaac: 传送到任意 (x, y, yaw)（自动化测试用）


# ---- 编码 ----

def encode_cmd_vel(vx: float, vy: float, wz: float) -> bytes:
    """编码 CMD_VEL 消息"""
    payload = {"vx": float(vx), "vy": float(vy), "wz": float(wz)}
    payload_bytes = msgpack.packb(payload, use_bin_type=True)
    header = struct.pack(HEADER_FMT, MAGIC, MSG_CMD_VEL, len(payload_bytes))
    return header + payload_bytes


def encode_reset(to_start: bool = True) -> bytes:
    """编码 RESET 消息
    Args:
        to_start: True = 回 config 起点 (yaw=0); False = 原地恢复 (保留 xy/yaw, 清 roll/pitch)
    """
    payload = {"to_start": bool(to_start)}
    payload_bytes = msgpack.packb(payload, use_bin_type=True)
    header = struct.pack(HEADER_FMT, MAGIC, MSG_RESET, len(payload_bytes))
    return header + payload_bytes


def encode_set_pose(x: float, y: float, yaw: float, z=None) -> bytes:
    """编码 SET_POSE 消息 — 把机器人传送到任意 (x, y, yaw)。

    自动化回归需要把机器人分别放到平地/坡道/台阶等不同起点跑 N 次；
    只有"回 config 起点"的话每个用例都得重启仿真（冷启 ~80s）。

    z=None 时服务端向下 raycast 找地面（推荐：坡道/平台上写死 z 会穿模或悬空）。
    """
    payload = {"x": float(x), "y": float(y), "yaw": float(yaw)}
    if z is not None:
        payload["z"] = float(z)
    payload_bytes = msgpack.packb(payload, use_bin_type=True)
    header = struct.pack(HEADER_FMT, MAGIC, MSG_SET_POSE, len(payload_bytes))
    return header + payload_bytes


# ---- 解码 ----

def decode_header(data: bytes):
    """解码消息头 → (magic, msg_type, payload_len) or None"""
    if len(data) < HEADER_SIZE:
        return None
    magic, msg_type, payload_len = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    if magic != MAGIC:
        return None
    return magic, msg_type, payload_len


def decode_payload(msg_type: int, payload_bytes: bytes) -> dict:
    """解码 msgpack payload"""
    data = msgpack.unpackb(payload_bytes, raw=False)
    if msg_type == MSG_POINTCLOUD and "xyz" in data:
        n = data["n_points"]
        data["xyz"] = np.frombuffer(data["xyz"], dtype=np.float32).reshape(n, 3)
    return data


def decode_message(buffer: bytes):
    """解码一条完整消息 → (msg_type, payload_dict, consumed) or (None, None, 0)"""
    hdr = decode_header(buffer)
    if hdr is None:
        return None, None, 0
    _, msg_type, payload_len = hdr
    total = HEADER_SIZE + payload_len
    if len(buffer) < total:
        return None, None, 0
    payload = decode_payload(msg_type, buffer[HEADER_SIZE:total])
    return msg_type, payload, total
