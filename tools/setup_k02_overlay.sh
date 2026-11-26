#!/bin/bash
# ============================================================
# k02_nav overlay setup (容器内运行)
# ============================================================
# 在 /k02_ws 建立独立 catkin overlay，用 symlink 把 k02_nav
# 仓库内需要的子包链接进来。当前只链接 grid_mapper（Step 3 验证）。
# 后续 P4/P5 增加包，只需在 PKGS 列表追加。
#
# 用法:
#   docker exec -it real2sim2real_ros1 bash /tools/setup_k02_overlay.sh
# 或容器内手动:
#   bash /tools/setup_k02_overlay.sh
# ============================================================
set -e

K02_SRC=/k02_nav/ros_ws/src
K02_WS=/k02_ws

# 当前 overlay 中需要编译的 k02_nav 子包列表
PKGS=(
    grid_mapper
    path_detect       # v2 Phase 1 Step A/B: 地面分割 + ground_map + scan/path_detect
    traversability    # v2 Phase 1 Step D: ground_map → passability_grid + scene_hint
    scan_filter       # v2 Phase 1 Step E: SLOPE 方向 scan ray 设 inf → /scan_filtered
    atomic_move       # v2 Phase 1 Step F: 近距 PID 精调 (GSM 末段调用)
    neupan_ros        # v2 Phase 1 Step F: NeuPAN local planner (依赖宿主 /k02_nav/NeuPAN python)
    map_planner       # v2 Phase 1 Step F: A*/CHOMP + goal_state_machine (依赖 atomic_move/grid_mapper/neupan_ros)
    passability_fuser # v2 Phase 1 Step G: /map/whole + /passability_grid -> /map/whole_traversable
    # v4 Phase 1 (2026-06-08): OctoMap + Traversability Analyzer 替换 path_detect
    pointcloud_filter         # 3D 点云预处理 (盲区/强度/视场角裁剪)
    octomap_occ               # 3D OcTree 占据图 + cliff_detector (cliff_topic 默认 remap 到 dummy)
    traversability_analyzer   # OcTree → support/cliff/observed → /traversability/passability_grid
)

if [ ! -d "${K02_SRC}" ]; then
    echo "[setup_k02_overlay] ERROR: ${K02_SRC} not found." >&2
    echo "  请确认 docker-compose.yml 已挂载 ../../k02_nav -> /k02_nav" >&2
    exit 1
fi

mkdir -p "${K02_WS}/src"

# 清理旧的 symlink，重新建（不删 build/devel，由 catkin_make 增量处理）
echo "[setup_k02_overlay] linking packages: ${PKGS[*]}"
for pkg in "${PKGS[@]}"; do
    src="${K02_SRC}/${pkg}"
    dst="${K02_WS}/src/${pkg}"
    if [ ! -d "${src}" ]; then
        echo "  WARN: ${src} not found, skip"
        continue
    fi
    # 如果已是同目标的 symlink 跳过，否则替换
    if [ -L "${dst}" ] && [ "$(readlink "${dst}")" = "${src}" ]; then
        echo "  ok   ${pkg} (already linked)"
    else
        rm -rf "${dst}"
        ln -s "${src}" "${dst}"
        echo "  link ${pkg}"
    fi
done

# 编译（依赖 /catkin_ws 的 sim_bridge 等的 setup.bash）
cd "${K02_WS}"
source /opt/ros/noetic/setup.bash
if [ -f /catkin_ws/devel/setup.bash ]; then
    source /catkin_ws/devel/setup.bash
fi

# 一律 --force-cmake: 开发期工具, 几秒 reconfigure 换取"新增 PKGS / 系统依赖变化
# 时不会被 CMakeCache 卡住"的鲁棒性. 没有 --force-cmake 时, 增量 catkin_make 看
# CMakeLists.txt 没变 → 跳过 cmake reconfigure → 新加的包 / 新装的系统库不被发现.
#
# CATKIN_WHITELIST_PACKAGES="" 是显式清空 whitelist, 防止历史残留 (某次手动
# -DCATKIN_WHITELIST_PACKAGES=xxx 会持久写入 CMakeCache, 导致后续只编子集).
echo "[setup_k02_overlay] catkin_make --force-cmake in ${K02_WS}"
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release -DCATKIN_WHITELIST_PACKAGES=""

# NeuPAN python package: editable install, 让 neupan_node.py 能 `import neupan`.
# /k02_nav 是 bind-mount, 路径在容器启动后才有, 所以必须在这里装而不能进 Dockerfile.
# --no-deps: NeuPAN 运行时依赖已在 Dockerfile.ros1 用 pip 装好, 不让 pip 再 resolve.
# --ignore-requires-python: pyproject.toml 写 >=3.10, 但实测 3.8 (Noetic 默认) 可跑.
# 幂等: 已安装时 pip 视作 no-op.
NEUPAN_PKG_DIR=/k02_nav/NeuPAN
if [ -f "${NEUPAN_PKG_DIR}/pyproject.toml" ]; then
    # 容器自带 pip 20.0.2 不支持 PEP 660 (pyproject.toml-only editable), 必须 >= 21.3.
    # 幂等: 已是新版时 pip 直接 no-op.
    PIP_MAJOR=$(pip3 --version 2>/dev/null | awk '{print $2}' | cut -d. -f1)
    if [ -z "${PIP_MAJOR}" ] || [ "${PIP_MAJOR}" -lt 22 ]; then
        echo "[setup_k02_overlay] upgrading pip (>= 21.3 needed for PEP 660 editable)"
        pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple --upgrade 'pip>=21.3' >/dev/null
    fi

    # 镜像 docker-ros1 是 2026-05-13 build, Dockerfile.ros1 后来加的 pip 包 (torch/cvxpy/...)
    # 在已存在镜像里不会自动生效. 这里做"运行时补齐", pip 已装会自动跳过. 持久化请 rebuild:
    #   docker compose -f docker/docker-compose.yml build ros1
    if ! python3 -c "import torch, cvxpy, scs, sklearn" 2>/dev/null; then
        echo "[setup_k02_overlay] installing NeuPAN runtime deps (torch CPU wheel + cvxpy 等)"
        # torch 走 CPU wheel 避免 GPU 2GB+ 下载超时
        pip3 install --index-url https://download.pytorch.org/whl/cpu 'torch<2.5' torchvision
        # 其余走清华源 (Py3.8 兼容性钉版本, 与 Dockerfile.ros1 第 30-36 行一致)
        pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
            msgpack pyyaml rich dill colorama \
            "cvxpy<1.6" cvxpylayers ecos "scs<=3.2.7" "scikit-learn<1.4"
    fi

    # gctl 1.2 顶层 import tkinter, 需要系统 python3-tk
    python3 -c "import tkinter" 2>/dev/null || {
        echo "[setup_k02_overlay] apt install python3-tk (gctl 1.2 顶层 import 依赖)"
        apt-get install -y --no-install-recommends python3-tk >/dev/null
    }

    echo "[setup_k02_overlay] pip install -e ${NEUPAN_PKG_DIR} (NeuPAN python pkg)"
    pip3 install --no-deps --ignore-requires-python -e "${NEUPAN_PKG_DIR}"
else
    echo "[setup_k02_overlay] WARN: ${NEUPAN_PKG_DIR}/pyproject.toml not found, skip NeuPAN pip install"
fi

echo
echo "[setup_k02_overlay] Done. To use in current shell:"
echo "  source ${K02_WS}/devel/setup.bash"
