"""
tcp_server.py
非阻塞 TCP 桥接服务端 — 在 Isaac Sim 主循环中调用, 不阻塞仿真.

用法:
    server = TcpBridgeServer(host="0.0.0.0", port=9090)
    server.start()

    # 在仿真主循环中:
    server.send_pose(x, y, z, qx, qy, qz, qw, ...)
    server.send_pointcloud(xyz_array)
    cmd = server.get_latest_cmd_vel()  # 非阻塞, 返回 None 或 (vx, vy, wz)

    server.stop()
"""

import socket
import selectors
import threading
import time
from collections import deque

from r2s2r.bridge.protocol import (
    encode_pose, encode_pointcloud, encode_image, encode_imu, encode_clock,
    decode_message, MSG_CMD_VEL, MSG_RESET,
    HEADER_SIZE,
)


class TcpBridgeServer:
    """非阻塞 TCP 服务端, 支持单客户端连接."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9090):
        self._host = host
        self._port = port
        self._running = False
        self._thread = None

        # 客户端连接
        self._client_sock = None
        self._client_lock = threading.Lock()

        # 接收缓冲区
        self._recv_buffer = bytearray()

        # 最新 cmd_vel (线程安全)
        self._cmd_vel = None  # (vx, vy, wz) or None
        self._cmd_vel_time = 0.0
        self._cmd_vel_lock = threading.Lock()

        # 复位请求 (线程安全, 一次性消费)
        # None = 无请求; True = 回起点; False = 原地恢复
        self._reset_request = None
        self._reset_lock = threading.Lock()

        # 发送队列 (主线程写, IO 线程发)
        self._send_queue = deque(maxlen=100)
        self._write_buf = b""  # 非阻塞写缓冲区 (部分发送残余)

    def start(self):
        """启动服务端 (后台线程)"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()
        print(f"[TcpBridgeServer] Listening on {self._host}:{self._port}")

    def stop(self):
        """停止服务端"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    @property
    def connected(self) -> bool:
        """是否有客户端连接"""
        with self._client_lock:
            return self._client_sock is not None

    def send_clock(self, timestamp):
        """Queue the simulation clock before observations from this step."""
        self._send_queue.append(encode_clock(timestamp))

    def send_pose(self, x, y, z, qx, qy, qz, qw,
                  vx=0.0, vy=0.0, vz=0.0,
                  wx=0.0, wy=0.0, wz=0.0,
                  timestamp=None):
        """发送 POSE 消息 (非阻塞, 写入发送队列)"""
        msg = encode_pose(x, y, z, qx, qy, qz, qw,
                          vx, vy, vz, wx, wy, wz, timestamp)
        self._send_queue.append(msg)

    def send_pointcloud(self, xyz, frame_id="map", timestamp=None, body_pose=None):
        """发送 POINTCLOUD 消息 (非阻塞)"""
        if xyz is None or len(xyz) == 0:
            return
        msg = encode_pointcloud(xyz, frame_id, timestamp, body_pose)
        self._send_queue.append(msg)

    def send_image(self, rgb, frame_id="camera", quality=80, timestamp=None,
                   intrinsics=None, body_to_camera=None):
        """发送 IMAGE 消息 (JPEG 压缩, 非阻塞)"""
        if rgb is None or rgb.size == 0:
            return
        msg = encode_image(rgb, frame_id, quality, timestamp,
                           intrinsics=intrinsics, body_to_camera=body_to_camera)
        self._send_queue.append(msg)

    def send_imu(self, lin_acc, ang_vel, orientation=None,
                 frame_id="imu", timestamp=None):
        """发送 IMU 消息 (非阻塞)"""
        if lin_acc is None or ang_vel is None:
            return
        msg = encode_imu(lin_acc, ang_vel, orientation, frame_id, timestamp)
        self._send_queue.append(msg)

    def get_latest_cmd_vel(self, timeout: float = 0.5):
        """获取最新的 cmd_vel 指令
        Args:
            timeout: 超时时间 (秒), 超过此时间无新指令返回 None
        Returns:
            (vx, vy, wz) or None
        """
        with self._cmd_vel_lock:
            if self._cmd_vel is None:
                return None
            if time.time() - self._cmd_vel_time > timeout:
                return None
            return self._cmd_vel

    def get_reset_requested(self):
        """检查并消费复位请求 (一次性)

        Returns:
            None: 无请求
            True: 回起点复位 (ROS 端 /sim_reset 触发)
            False: 原地恢复复位
        """
        with self._reset_lock:
            req = self._reset_request
            self._reset_request = None
            return req

    # ---- 内部 IO 线程 ----

    def _io_loop(self):
        """后台 IO 循环: accept + recv + send"""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self._host, self._port))
        server_sock.listen(1)
        server_sock.setblocking(False)

        sel = selectors.DefaultSelector()
        sel.register(server_sock, selectors.EVENT_READ)

        try:
            while self._running:
                events = sel.select(timeout=0.01)

                for key, mask in events:
                    if key.fileobj is server_sock:
                        self._handle_accept(server_sock, sel)
                    else:
                        self._handle_client_read(key.fileobj, sel)

                # 发送队列中的数据
                self._flush_send_queue()

        finally:
            sel.close()
            with self._client_lock:
                if self._client_sock:
                    self._client_sock.close()
                    self._client_sock = None
            server_sock.close()

    def _handle_accept(self, server_sock, sel):
        """处理新客户端连接"""
        try:
            conn, addr = server_sock.accept()
            conn.setblocking(False)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            with self._client_lock:
                # 关闭旧连接
                if self._client_sock:
                    try:
                        sel.unregister(self._client_sock)
                    except Exception:
                        pass
                    self._client_sock.close()
                self._client_sock = conn

            sel.register(conn, selectors.EVENT_READ)
            self._recv_buffer.clear()
            print(f"[TcpBridgeServer] Client connected: {addr}")

        except Exception as e:
            print(f"[TcpBridgeServer] Accept error: {e}")

    def _handle_client_read(self, sock, sel):
        """处理客户端数据"""
        try:
            data = sock.recv(4096)
            if not data:
                # 客户端断开
                self._handle_disconnect(sock, sel)
                return

            self._recv_buffer.extend(data)

            # 解析所有完整消息
            while True:
                msg_type, payload, consumed = decode_message(bytes(self._recv_buffer))
                if msg_type is None:
                    break
                self._recv_buffer = self._recv_buffer[consumed:]

                if msg_type == MSG_CMD_VEL:
                    vx = payload.get("vx", 0.0)
                    vy = payload.get("vy", 0.0)
                    wz = payload.get("wz", 0.0)
                    with self._cmd_vel_lock:
                        self._cmd_vel = (vx, vy, wz)
                        self._cmd_vel_time = time.time()
                    # 首次收到或值变化时打印
                    if not hasattr(self, '_last_logged_cmd') or \
                       self._last_logged_cmd != (vx, vy, wz):
                        print(f"[TcpBridgeServer] CMD_VEL received: vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
                        self._last_logged_cmd = (vx, vy, wz)

                elif msg_type == MSG_RESET:
                    to_start = bool(payload.get("to_start", True))
                    with self._reset_lock:
                        self._reset_request = to_start
                    print(f"[TcpBridgeServer] RESET received: "
                          f"{'to_start' if to_start else 'in_place'}")

        except (ConnectionResetError, BrokenPipeError, OSError):
            self._handle_disconnect(sock, sel)

    def _handle_disconnect(self, sock, sel):
        """处理客户端断开"""
        try:
            sel.unregister(sock)
        except Exception:
            pass
        sock.close()
        with self._client_lock:
            if self._client_sock is sock:
                self._client_sock = None
        self._recv_buffer.clear()
        self._write_buf = b""
        print("[TcpBridgeServer] Client disconnected")

    def _flush_send_queue(self):
        """Non-blocking send: 每次尽量发, 不阻塞 IO 线程"""
        with self._client_lock:
            sock = self._client_sock
        if sock is None:
            self._send_queue.clear()
            self._write_buf = b""
            return

        # Drain ready messages so adding /clock cannot build a per-message backlog.
        while self._write_buf or self._send_queue:
            if not self._write_buf:
                self._write_buf = self._send_queue.popleft()
            try:
                sent = sock.send(self._write_buf)
                if sent <= 0:
                    return
                self._write_buf = self._write_buf[sent:]
            except BlockingIOError:
                return  # Retain partially sent frames until the socket is writable.
            except (ConnectionResetError, BrokenPipeError, OSError):
                self._write_buf = b""
                self._send_queue.clear()
                return
