"""真机差距注入模型（纯 Python，不依赖 ROS）。

桥节点关闭注入时完全不实例化这些类；开启时按通道独立工作。
所有随机量由 seed 派生，实验可复现。
"""

from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass

import numpy as np


class DelayQueue:
    """按到期时刻消费的 FIFO。调用方保证 push 的 due 单调不减。"""

    def __init__(self):
        self._q = deque()

    def push(self, due: float, item) -> None:
        self._q.append((due, item))

    def pop_due(self, now: float) -> list:
        out = []
        while self._q and self._q[0][0] <= now:
            out.append(self._q.popleft()[1])
        return out

    def __len__(self) -> int:
        return len(self._q)


@dataclass
class CmdImpairConfig:
    enable: bool = True
    delay_ms: float = 0.0
    tau_s: float = 0.0
    deadband_v: float = 0.0
    deadband_w_rest: float = 0.0
    deadband_w_move: float = 0.0
    w_move_eps: float = 0.05
    v_max: float = 1e9
    vy_max: float = 1e9
    w_max: float = 1e9


class CmdImpair:
    def __init__(self, cfg: CmdImpairConfig, seed: int = 0):
        self.cfg = cfg
        self._q = DelayQueue()
        self._target = (0.0, 0.0, 0.0)
        self._applied = (0.0, 0.0, 0.0)
        self._last_t = None

    def update(self, cmd, now: float):
        """cmd=None 表示本周期无新指令，只推进延迟/惯性。"""
        if cmd is not None:
            self._q.push(now + self.cfg.delay_ms * 1e-3,
                         tuple(float(v) for v in cmd))
        for item in self._q.pop_due(now):
            self._target = item
        tgt = self._deadband(self._target)
        if self.cfg.tau_s > 0.0 and self._last_t is not None:
            dt = max(0.0, now - self._last_t)
            a = dt / (self.cfg.tau_s + dt)
            self._applied = tuple(a * t + (1.0 - a) * p
                                  for t, p in zip(tgt, self._applied))
        else:
            self._applied = tgt
        self._last_t = now
        return self._saturate(self._applied)

    def queue_len(self) -> int:
        return len(self._q)

    def _deadband(self, cmd):
        vx, vy, wz = cmd
        if math.hypot(vx, vy) < self.cfg.deadband_v:
            vx = vy = 0.0
        moving = math.hypot(vx, vy) >= self.cfg.w_move_eps
        w_db = self.cfg.deadband_w_move if moving else self.cfg.deadband_w_rest
        if abs(wz) < w_db:
            wz = 0.0
        return (vx, vy, wz)

    def _saturate(self, cmd):
        vx, vy, wz = cmd
        return (max(-self.cfg.v_max, min(self.cfg.v_max, vx)),
                max(-self.cfg.vy_max, min(self.cfg.vy_max, vy)),
                max(-self.cfg.w_max, min(self.cfg.w_max, wz)))


@dataclass
class OdomImpairConfig:
    enable: bool = True
    delay_ms: float = 0.0
    noise_xy: float = 0.0
    noise_yaw: float = 0.0
    noise_v: float = 0.0
    noise_w: float = 0.0
    drift_std_xy: float = 0.0
    drift_tau_s: float = 20.0
    jump_rate_hz: float = 0.0
    jump_std_xy: float = 0.05
    jump_std_yaw: float = 0.03
    drop_prob: float = 0.0
    rigid_cloud: bool = False


def _quat_mul_yaw(qx, qy, qz, qw, dyaw):
    s, c = math.sin(dyaw / 2.0), math.cos(dyaw / 2.0)
    return (qx * c + qy * s,
            -qx * s + qy * c,
            qw * s + qz * c,
            qw * c - qz * s)


class OdomImpair:
    def __init__(self, cfg: OdomImpairConfig, seed: int = 0):
        self.cfg = cfg
        self._rng = random.Random(seed)
        self._drift = [0.0, 0.0]
        self._last_t = None
        self._q = DelayQueue()

    def current_drift(self):
        return (self._drift[0], self._drift[1])

    def should_drop(self):
        return self.cfg.drop_prob > 0.0 and self._rng.random() < self.cfg.drop_prob

    def process(self, d: dict, now: float):
        if self.should_drop():
            return []
        self._q.push(now + self.cfg.delay_ms * 1e-3, self.distort(d, now))
        return self._q.pop_due(now)

    def flush(self, now: float):
        return self._q.pop_due(now)

    def queue_len(self) -> int:
        return len(self._q)

    def distort(self, d: dict, now: float):
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now
        if self.cfg.drift_std_xy > 0.0 and dt > 0.0:
            tau = max(1e-3, self.cfg.drift_tau_s)
            a = math.exp(-dt / tau)
            b = self.cfg.drift_std_xy * math.sqrt(max(0.0, 1.0 - a * a))
            self._drift[0] = a * self._drift[0] + b * self._rng.gauss(0.0, 1.0)
            self._drift[1] = a * self._drift[1] + b * self._rng.gauss(0.0, 1.0)
        out = dict(d)
        out["x"] = float(d["x"]) + self._drift[0] + self._rng.gauss(0.0, self.cfg.noise_xy)
        out["y"] = float(d["y"]) + self._drift[1] + self._rng.gauss(0.0, self.cfg.noise_xy)
        dyaw = self._rng.gauss(0.0, self.cfg.noise_yaw)
        if self.cfg.jump_rate_hz > 0.0:
            p = 1.0 - math.exp(-self.cfg.jump_rate_hz * max(dt, 0.01))
            if self._rng.random() < p:
                out["x"] += self._rng.gauss(0.0, self.cfg.jump_std_xy)
                out["y"] += self._rng.gauss(0.0, self.cfg.jump_std_xy)
                dyaw += self._rng.gauss(0.0, self.cfg.jump_std_yaw)
        qx, qy, qz, qw = (float(d.get(k, 0.0)) for k in ("qx", "qy", "qz", "qw"))
        if dyaw != 0.0:
            qx, qy, qz, qw = _quat_mul_yaw(qx, qy, qz, qw, dyaw)
        out.update(qx=qx, qy=qy, qz=qz, qw=qw)
        out["vx"] = float(d.get("vx", 0.0)) + self._rng.gauss(0.0, self.cfg.noise_v)
        out["vy"] = float(d.get("vy", 0.0)) + self._rng.gauss(0.0, self.cfg.noise_v)
        out["wz"] = float(d.get("wz", 0.0)) + self._rng.gauss(0.0, self.cfg.noise_w)
        return out


@dataclass
class CloudImpairConfig:
    enable: bool = True
    rate_hz: float = 0.0
    delay_ms: float = 0.0
    frame_drop_prob: float = 0.0
    point_drop_prob: float = 0.0
    noise_xyz: float = 0.0
    outlier_prob: float = 0.0
    outlier_radius_min: float = 0.5
    outlier_radius_max: float = 5.0
    stamp_jitter_ms: float = 0.0


class CloudImpair:
    def __init__(self, cfg: CloudImpairConfig, seed: int = 0):
        self.cfg = cfg
        self._rng = random.Random(seed)
        self._nprng = np.random.default_rng(seed + 1)
        self._last_accept = None
        self._q = DelayQueue()
        self._frames_in = 0
        self._frames_out = 0
        self._points_in = 0
        self._points_out = 0

    def stats(self):
        return {
            "frames_in": self._frames_in,
            "frames_out": self._frames_out,
            "points_in": self._points_in,
            "points_out": self._points_out,
            "queue": len(self._q),
        }

    def queue_len(self) -> int:
        return len(self._q)

    def process(self, d: dict, now: float):
        if not self.cfg.enable:
            return [d]
        self._frames_in += 1
        if self.cfg.rate_hz > 0.0 and self._last_accept is not None:
            if now - self._last_accept < 1.0 / self.cfg.rate_hz:
                return self._q.pop_due(now)
        if self.cfg.frame_drop_prob > 0.0 and self._rng.random() < self.cfg.frame_drop_prob:
            self._last_accept = now
            return self._q.pop_due(now)
        self._last_accept = now
        out = dict(d)
        xyz = d.get("xyz")
        if xyz is not None:
            arr = np.asarray(xyz, dtype=np.float32).reshape(-1, 3).copy()
            self._points_in += len(arr)
            if self.cfg.point_drop_prob > 0.0 and len(arr):
                keep = self._nprng.random(len(arr)) >= self.cfg.point_drop_prob
                arr = arr[keep]
            if self.cfg.noise_xyz > 0.0 and len(arr):
                arr += self._nprng.normal(0.0, self.cfg.noise_xyz, arr.shape).astype(np.float32)
            if self.cfg.outlier_prob > 0.0 and len(arr):
                n_out = int(round(self.cfg.outlier_prob * len(arr)))
                if n_out > 0:
                    idx = self._nprng.choice(len(arr), size=min(n_out, len(arr)), replace=False)
                    r = self._nprng.uniform(self.cfg.outlier_radius_min,
                                            self.cfg.outlier_radius_max,
                                            size=(len(idx), 1))
                    dirs = self._nprng.normal(size=(len(idx), 3))
                    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True) + 1e-9
                    arr[idx] += (dirs * r).astype(np.float32)
            self._points_out += len(arr)
            out["xyz"] = arr
        if self.cfg.stamp_jitter_ms > 0.0 and "ts" in d:
            out["ts"] = float(d["ts"]) + self._rng.uniform(
                -self.cfg.stamp_jitter_ms, self.cfg.stamp_jitter_ms) * 1e-3
        self._frames_out += 1
        self._q.push(now + self.cfg.delay_ms * 1e-3, out)
        return self._q.pop_due(now)

    def flush(self, now: float):
        return self._q.pop_due(now)


@dataclass
class ImuImpairConfig:
    enable: bool = True
    noise_gyro: float = 0.0
    noise_accel: float = 0.0
    bias_gyro_z: float = 0.0


class ImuImpair:
    def __init__(self, cfg: ImuImpairConfig, seed: int = 0):
        self.cfg = cfg
        self._rng = random.Random(seed)

    def distort(self, d: dict):
        if not self.cfg.enable:
            return d
        out = dict(d)
        for k in ("ax", "ay", "az"):
            out[k] = float(d.get(k, 0.0)) + self._rng.gauss(0.0, self.cfg.noise_accel)
        for k in ("wx", "wy", "wz"):
            out[k] = float(d.get(k, 0.0)) + self._rng.gauss(0.0, self.cfg.noise_gyro)
        out["wz"] += self.cfg.bias_gyro_z
        return out


def make_profile(name: str):
    off = {
        "cmd": CmdImpairConfig(enable=False),
        "odom": OdomImpairConfig(enable=False),
        "cloud": CloudImpairConfig(enable=False),
        "imu": ImuImpairConfig(enable=False),
    }
    if name in ("", "off", "none"):
        return off
    if name == "mild":
        return {
            "cmd": CmdImpairConfig(delay_ms=50.0, tau_s=0.06),
            "odom": OdomImpairConfig(delay_ms=50.0, noise_xy=0.01,
                                     noise_yaw=0.005, drift_std_xy=0.01),
            "cloud": CloudImpairConfig(rate_hz=9.4, delay_ms=50.0,
                                       noise_xyz=0.01),
            "imu": ImuImpairConfig(),
        }
    if name == "severe":
        return {
            "cmd": CmdImpairConfig(delay_ms=120.0, tau_s=0.20,
                                   deadband_v=0.22, deadband_w_rest=0.45,
                                   deadband_w_move=0.25),
            "odom": OdomImpairConfig(delay_ms=150.0, noise_xy=0.03,
                                     noise_yaw=0.015, drift_std_xy=0.04,
                                     noise_v=0.02, noise_w=0.02,
                                     jump_rate_hz=0.02),
            "cloud": CloudImpairConfig(rate_hz=9.4, delay_ms=200.0,
                                       point_drop_prob=0.35,
                                       noise_xyz=0.025, outlier_prob=0.01),
            "imu": ImuImpairConfig(noise_gyro=0.01, noise_accel=0.05),
        }
    return {
        "cmd": CmdImpairConfig(delay_ms=80.0, tau_s=0.12,
                               deadband_v=0.22, deadband_w_rest=0.45,
                               deadband_w_move=0.20),
        "odom": OdomImpairConfig(delay_ms=100.0, noise_xy=0.02,
                                 noise_yaw=0.01, drift_std_xy=0.02,
                                 noise_v=0.01, noise_w=0.01),
        "cloud": CloudImpairConfig(rate_hz=9.4, delay_ms=100.0,
                                   point_drop_prob=0.20,
                                   noise_xyz=0.015, outlier_prob=0.003),
        "imu": ImuImpairConfig(),
    }
