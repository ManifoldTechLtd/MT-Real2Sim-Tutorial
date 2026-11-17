#!/bin/bash
# 抓取 Isaac Sim 卡死现场：C++/Python 全线程栈 + 系统状态
# 用法: ./capture_hang.sh [容器名]
# 说明: eu-stack 基于 .eh_frame 展开，不依赖调试符号；gdb/py-spy 用于 Python 侧。
#       抓取会暂停目标进程数秒到数分钟，建议只在确认已卡死时执行。
set -u

CTN=${1:-isaac_nav_isaac}
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BASE="${HANG_LOG_DIR:-$PROJECT_ROOT/hang_logs}"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$BASE/$STAMP"
mkdir -p "$OUT"
ln -sfn "$OUT" "$BASE/latest"

PID=$(docker exec "$CTN" pgrep -f 'python3 .*run_sim\.py' | head -1)
if [ -z "${PID}" ]; then
    echo "未找到 run_sim.py 的 python3 进程，容器状态："
    docker ps --format '{{.Names}}\t{{.Status}}' | grep isaac
    exit 1
fi
echo "[capture] container=$CTN pid=$PID out=$OUT"

# 工具自检（容器重建后会丢失，自动补装）
docker exec "$CTN" bash -lc '
command -v gdb >/dev/null || { apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gdb >/dev/null 2>&1; }
command -v eu-stack >/dev/null || { apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq elfutils >/dev/null 2>&1; }
command -v pip3 >/dev/null || { apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-pip >/dev/null 2>&1; }
command -v py-spy >/dev/null || pip3 install --break-system-packages -q py-spy >/dev/null 2>&1
' >/dev/null 2>&1

# 系统概况
{ date; uptime; free -h; echo; nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw,temperature.gpu --format=csv; } > "$OUT/sysinfo.txt" 2>&1

# 各线程状态（ps 的 %CPU 是生命周期均值，不可用于找自旋线程）
docker exec "$CTN" bash -lc "ps -T -p $PID -o tid,stat,pcpu,comm,wchan:24" > "$OUT/ps_threads.txt" 2>&1

# 按 3 秒增量找自旋线程（从宿主机直接读 /proc/<hostpid>/task）
HOST_PID=$(docker top "$CTN" -eo pid,args 2>/dev/null | grep 'python3 r2s2r/run_sim.py' | awk '{print $1}' | head -1)
if [ -n "${HOST_PID}" ]; then
    SNAPA=$(mktemp); SNAPB=$(mktemp)
    snap() {
        local t
        for t in /proc/$HOST_PID/task/[0-9]*; do
            [ -r "$t/stat" ] || continue
            printf '%s\t%s\t%s\t%s\n' "${t##*/}" \
                "$(awk '/^NSpid:/{print $NF}' "$t/status" 2>/dev/null)" \
                "$(tr -d '\n' < "$t/comm" 2>/dev/null)" \
                "$(awk '{sub(/^.*\) /,""); print $12+$13}' "$t/stat" 2>/dev/null)"
        done
    }
    snap > "$SNAPA"; sleep 3; snap > "$SNAPB"
    awk -F'\t' 'NR==FNR{a[$1]=$4; c[$1]=$2; n[$1]=$3; next}
                {d=$4-a[$1]; if (d>2) printf "%-7s ctr=%-6s %-22s +%d ticks/3s\n", $1, c[$1], n[$1], d}' \
        "$SNAPA" "$SNAPB" | sort -t'+' -k2 -rn | head -20 > "$OUT/thread_cpu.txt"
    rm -f "$SNAPA" "$SNAPB"
else
    echo "未找到宿主机 PID" > "$OUT/thread_cpu.txt"
fi

# C++ 全线程栈（带模块 build-id/偏移，可用 maps 解析）
docker exec "$CTN" bash -lc "timeout 120 eu-stack -p $PID -b" > "$OUT/eustack.txt" 2>&1

# 地址空间（地址→模块）
docker exec "$CTN" cat "/proc/$PID/maps" > "$OUT/maps.txt" 2>&1

# Python 栈
docker exec "$CTN" bash -lc "timeout 60 py-spy dump --pid $PID" > "$OUT/py_stacks.txt" 2>&1

# gdb 补充（Python/USD 符号友好，linux 符号可能不准；auto-load 必须放开）
docker exec "$CTN" bash -lc "timeout 240 gdb -p $PID -batch \
    -iex 'set auto-load safe-path /' -ex 'set pagination off' -ex 'set confirm off' \
    -ex 'thread apply all bt 12'" > "$OUT/gdb_bt.txt" 2>&1

# 进程内存
docker exec "$CTN" bash -lc "grep -E 'VmRSS|VmSwap|Threads' /proc/$PID/status" > "$OUT/proc_status.txt" 2>&1

# 桥接与 ROS 连接
ss -tnp 2>/dev/null | grep -E ':9090|:11311' > "$OUT/net.txt" 2>&1

# 仿真日志尾部与最后 diag
docker exec "$CTN" bash -lc '
F=$(ls -t "/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1/"kit_*.log 2>/dev/null | head -1)
echo "LOG=$F"
tail -n 300 "$F"
echo "--- last diag ---"
grep "\[diag\]" "$F" | tail -5
' > "$OUT/sim_log_tail.txt" 2>&1

echo "[capture] done"
head -8 "$OUT/ps_threads.txt" >&2
ls -lh "$OUT"