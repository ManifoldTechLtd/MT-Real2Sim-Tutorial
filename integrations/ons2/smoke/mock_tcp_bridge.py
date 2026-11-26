#!/usr/bin/env python3
"""Minimal Isaac TCP stand-in for ONS2 wiring checks without a GPU.

Speaks the same msgpack protocol as r2s2r/bridge/tcp_server.py:
  - sends MSG_CLOCK / MSG_POSE / MSG_POINTCLOUD at 30/30/5 Hz
  - integrates incoming MSG_CMD_VEL for a simple unicycle model

Usage:
  python3 mock_tcp_bridge.py [--host 0.0.0.0] [--port 9090] [--pose 0,0,0]
"""

import argparse
import math
import os
import socket
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "r2s2r", "bridge"))
from protocol import (MSG_CMD_VEL, MSG_IMU, HEADER_SIZE,  # noqa: E402
                      decode_header, encode_clock, encode_message,
                      encode_pointcloud, encode_pose)


class MockIsaac:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.vx = self.wz = 0.0
        self.lock = threading.Lock()
        self.t0 = time.time()

    def step(self, dt):
        with self.lock:
            self.x += (self.vx * math.cos(self.yaw) - 0.0 * math.sin(self.yaw)) * dt
            self.y += (self.vx * math.sin(self.yaw) + 0.0 * math.cos(self.yaw)) * dt
            self.yaw += self.wz * dt

    def pose_payload(self):
        half = self.yaw / 2.0
        return dict(x=self.x, y=self.y, z=0.12,
                    qx=0.0, qy=0.0, qz=math.sin(half), qw=math.cos(half),
                    vx=self.vx, vy=0.0, wz=self.wz)

    @staticmethod
    def cloud(n=720, radius=4.0):
        ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
        z = np.linspace(0.1, 1.0, 8)
        return np.stack([
            (radius * np.cos(ang))[:, None] * np.ones_like(z)[None, :],
            (radius * np.sin(ang))[:, None] * np.ones_like(z)[None, :],
            np.ones_like(ang)[:, None] * z[None, :],
        ], axis=-1).reshape(-1, 3).astype(np.float32)


def handle_client(conn, sim):
    conn.settimeout(0.2)
    buf = bytearray()
    t_last = time.time()

    def send_all(payload):
        conn.sendall(payload)

    next_pose = next_cloud = next_imu = 0.0
    try:
        while True:
            now = time.time()
            dt = now - t_last
            t_last = now
            sim.step(dt)
            ts = now - sim.t0
            if now >= next_pose:
                send_all(encode_clock(ts))
                p = sim.pose_payload()
                send_all(encode_pose(p['x'], p['y'], p['z'], p['qx'], p['qy'],
                                     p['qz'], p['qw'], vx=p['vx'], wz=p['wz'],
                                     timestamp=ts))
                next_pose = now + 1 / 30.0
            if now >= next_imu:
                send_all(encode_message(MSG_IMU, {'ts': ts, 'frame_id': 'imu'}))
                next_imu = now + 1 / 30.0
            if now >= next_cloud:
                p = sim.pose_payload()
                send_all(encode_pointcloud(
                    sim.cloud(), frame_id='map', timestamp=ts,
                    body_pose=[p['x'], p['y'], p['z'], p['qx'], p['qy'], p['qz'], p['qw']]))
                next_cloud = now + 5.0
            try:
                data = conn.recv(65536)
                if not data:
                    return
                buf.extend(data)
            except socket.timeout:
                pass
            while True:
                hdr = decode_header(buf)
                if hdr is None:
                    break
                _, mtype, plen = hdr
                total = HEADER_SIZE + plen
                if len(buf) < total:
                    break
                if mtype == MSG_CMD_VEL:
                    import msgpack
                    p = msgpack.unpackb(bytes(buf[HEADER_SIZE:total]), raw=False)
                    with sim.lock:
                        sim.vx = float(p.get('vx', 0.0))
                        sim.wz = float(p.get('wz', 0.0))
                del buf[:total]
    except (ConnectionResetError, BrokenPipeError, OSError):
        pass
    finally:
        conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--pose', default='0,0,0', help='x,y,yaw')
    args = ap.parse_args()
    x, y, yaw = (float(v) for v in args.pose.split(','))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f'[mock_tcp] listening on {args.host}:{args.port} pose=({x},{y},{yaw})')
    while True:
        conn, addr = srv.accept()
        print(f'[mock_tcp] client {addr}')
        sim = MockIsaac(x, y, yaw)
        threading.Thread(target=handle_client, args=(conn, sim), daemon=True).start()


if __name__ == '__main__':
    main()
