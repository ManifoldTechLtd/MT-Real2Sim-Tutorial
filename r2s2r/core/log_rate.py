"""打印节流: 同一 key 每 period 秒最多输出一次, 并附上期间被压掉的条数."""

import time

_last = {}
_skipped = {}


def log_throttled(key, msg, period=2.0):
    now = time.time()
    if now - _last.get(key, 0.0) < period:
        _skipped[key] = _skipped.get(key, 0) + 1
        return
    dropped = _skipped.pop(key, 0)
    _last[key] = now
    print(f"{msg} (+{dropped} suppressed)" if dropped else msg)
