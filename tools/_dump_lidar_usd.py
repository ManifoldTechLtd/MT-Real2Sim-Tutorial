#!/usr/bin/env python3
"""Dump OmniLidar USD 属性 (调试用). 需启动 SimulationApp 才能 import pxr.

用法: /isaac-sim/python.sh _dump_lidar_usd.py <usd1> [usd2 ...]
数组属性显示 len/min/max, 用于核对发射器 FOV 是否写入正确.

注意: Isaac Sim 启动后 stdout 的 print 会被吞掉, 故结果同时写入
/workspace/lidar_configs/_usd_dump.txt (rw 挂载, 宿主机可读).
"""
import sys
from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})
from pxr import Usd  # noqa: E402

_OUT = []


def emit(s):
    _OUT.append(s)
    print(s, flush=True)


def dump(path):
    stage = Usd.Stage.Open(path)
    emit("=== USD: %s ===" % path)
    for prim in stage.Traverse():
        attrs = [a for a in prim.GetAttributes() if a.Get() is not None]
        if not attrs:
            continue
        emit("PRIM: %s type=%s" % (prim.GetPath(), prim.GetTypeName()))
        for a in attrs:
            name = a.GetName()
            v = a.Get()
            if hasattr(v, '__len__') and not isinstance(v, str):
                n = len(v)
                nums = [x for x in v if isinstance(x, (int, float))]
                if nums and n > 4:
                    emit("   %-50s len=%-7d min=%.4f max=%.4f"
                         % (name, n, min(nums), max(nums)))
                else:
                    emit("   %-50s len=%-7d sample=%s" % (name, n, list(v)[:4]))
            else:
                emit("   %-50s = %s" % (name, v))


for _p in sys.argv[1:]:
    dump(_p)

# 写结果到 rw 挂载目录, 宿主机可读 (绕过 Isaac Sim 对 stdout 的重定向)
try:
    with open("/workspace/lidar_configs/_usd_dump.txt", "w") as f:
        f.write("\n".join(_OUT) + "\n")
except Exception as e:  # noqa: BLE001
    print("WRITE FAIL:", e, flush=True)

_app.close()
