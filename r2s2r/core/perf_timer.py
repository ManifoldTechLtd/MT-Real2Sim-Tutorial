"""
perf_timer.py
轻量级性能计时工具 — 用于定位主循环中的耗时热点.

用法:
    from r2s2r.core.perf_timer import PerfTimer
    timer = PerfTimer()
    with timer("section_name"):
        do_work()
    # 每 N 帧打印
    if frame % 100 == 0:
        print(timer.summary())
        timer.reset()
"""

import time
from collections import defaultdict


class PerfTimer:
    """命名计时器集合, 支持 context manager."""

    def __init__(self):
        self._stats = defaultdict(lambda: {
            "count": 0, "total": 0.0,
            "min": float("inf"), "max": 0.0,
        })

    class _Context:
        __slots__ = ("_timer", "_name", "_t0")

        def __init__(self, timer, name):
            self._timer = timer
            self._name = name
            self._t0 = 0.0

        def __enter__(self):
            self._t0 = time.perf_counter()
            return self

        def __exit__(self, *exc):
            elapsed = (time.perf_counter() - self._t0) * 1000.0  # ms
            s = self._timer._stats[self._name]
            s["count"] += 1
            s["total"] += elapsed
            s["min"] = min(s["min"], elapsed)
            s["max"] = max(s["max"], elapsed)
            return False

    def __call__(self, name: str):
        """用作 context manager: with timer("name"): ..."""
        return self._Context(self, name)

    def record(self, name: str, elapsed_ms: float):
        """手动记录一次耗时 (ms)."""
        s = self._stats[name]
        s["count"] += 1
        s["total"] += elapsed_ms
        s["min"] = min(s["min"], elapsed_ms)
        s["max"] = max(s["max"], elapsed_ms)

    def reset(self):
        self._stats.clear()

    def summary(self, title: str = "PerfTimer") -> str:
        if not self._stats:
            return f"[{title}] (no data)"
        lines = [
            f"\n{'='*72}",
            f" [{title}] Performance Summary",
            f"{'='*72}",
            f"  {'Section':<30s} {'Count':>5s} {'Avg':>8s} {'Min':>8s} {'Max':>8s} {'Total':>9s}",
            f"  {'-'*30} {'-'*5} {'-'*8} {'-'*8} {'-'*8} {'-'*9}",
        ]
        sorted_items = sorted(
            self._stats.items(), key=lambda x: x[1]["total"], reverse=True)
        for name, s in sorted_items:
            count = s["count"]
            total = s["total"]
            avg = total / count if count > 0 else 0
            mn = s["min"] if s["min"] != float("inf") else 0
            mx = s["max"]
            lines.append(
                f"  {name:<30s} {count:>5d} {avg:>7.2f}ms {mn:>7.2f}ms {mx:>7.2f}ms {total:>8.1f}ms"
            )
        lines.append(f"{'='*72}")
        return "\n".join(lines)


# ---- Null timer (zero overhead when profiling disabled) ----

class _NullCtx:
    __slots__ = ()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class _NullTimer:
    _ctx = _NullCtx()

    def __call__(self, name):
        return self._ctx

    def record(self, name, ms):
        pass

    def reset(self):
        pass

    def summary(self, title=""):
        return ""


NULL_TIMER = _NullTimer()
