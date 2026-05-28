# 用户指南 — Isaac Real2Sim2Real Go2

[English](user_guide.md)

四足机器人仿真平台。一键启动 Isaac Sim + ROS + RViz，用键盘控制 Go2 机器人在 3D 场景中行走。

---

## 0. 新机器部署 Checklist (TL;DR)

> 这是给"拿到代码就部署"的同学的清单。每一项链接到本文对应小节有详细步骤与排错。

| # | 任务 | 章节 | 完成判据 |
|---|------|------|----------|
| 1 | 装 Docker + 加入 docker 组 + **重新登录** | §2.1 | `groups` 含 `docker` |
| 2 | （中国大陆）配置 Docker 镜像加速器 | §2.1.★ | `docker pull hello-world` 不超时 |
| 3 | 装 NVIDIA 驱动 + **重启** | §2.2 | `nvidia-smi` 输出 GPU 信息 |
| 4 | 装 nvidia-container-toolkit + 注意 daemon.json 合并 | §2.3 | `docker run --gpus all nvidia/cuda:... nvidia-smi` 工作 |
| 5 | （非必需）NGC 登录 | §3 | `docker manifest inspect nvcr.io/nvidia/isaac-sim:5.1.0` 不报 401 |
| 6 | 放场景文件到 `~/dataset/isaac/`，目录所有权确认 | §4 | `ls ~/dataset/isaac/*.usdz` 有文件 |
| 7 | 修改 `r2s2r/configs/config.yaml` 的 `usdz_path` | §4 | 路径以 `~/dataset/isaac/` 开头 |
| 8 | 首次启动用大超时 | §5 | `MAX_WAIT=1800 ./start.sh` |
| 9 | 启动后验证 ROS 通讯 | §5.4 | `rostopic list` 包含 `/cmd_vel /registered_scan` 等 |

> ⏱️ **总时间预估**：环境装好后，首次启动 ≈ **30–50 分钟**（镜像拉取 20GB + shader 编译）；后续启动 ≈ **1 分钟**。

---

## 1. 系统要求

| 项目 | 要求 |
|------|------|
| 系统 | Linux x86_64 (Ubuntu 22.04 / 24.04) |
| GPU | NVIDIA RTX 3070+ (显存 >= 8GB), 推荐 RTX 4080+ |
| 驱动 | NVIDIA >= 560 (推荐 580+); RTX 50系 (Blackwell) 需 `-open` 变体 |
| 内存 | >= 32GB |
| 磁盘 | >= 30GB 可用空间 |

> ⚠️ **显存提示**：8GB 显存 (如 RTX 5060 Mobile / 4060 / 3060) 是 Isaac Sim 5.1 的最低门槛。运行时请关闭浏览器、IDE 以外的 GPU 应用，避免 OOM。

---

## 2. 安装 Docker

### 2.1 安装 Docker

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-v2

# 当前用户免 sudo
sudo usermod -aG docker $USER
```

> ⚠️ 加入 docker 组后 **必须注销重新登录**（或在当前 shell 执行 `newgrp docker`）才会生效。用 `groups` 确认输出里含 `docker`。

验证:
```bash
docker --version
docker compose version
docker info | grep "Server Version"   # 能拿到输出说明无需 sudo 即可访问
```

### 2.1.★ 国内镜像加速器 (中国大陆用户必设)

项目依赖 Docker Hub 上的 `ros:noetic-perception` 镜像。国内直连 `registry-1.docker.io` 几乎必超时，需配置镜像加速器。

创建或编辑 `/etc/docker/daemon.json`（若后面装 nvidia-container-toolkit 后这个文件会被重写，可等那一步后再添加 `registry-mirrors` 字段）：

```bash
sudo tee /etc/docker/daemon.json >/dev/null <<'EOF'
{
    "registry-mirrors": [
        "https://docker.m.daocloud.io",
        "https://docker.1ms.run",
        "https://dockerproxy.com"
    ]
}
EOF
sudo systemctl restart docker
```

验证拉取:
```bash
docker pull hello-world && docker rmi hello-world
```

### 2.2 安装 NVIDIA 驱动

查询推荐驱动：
```bash
ubuntu-drivers devices
```

选择 **`nvidia-driver-<版本>-open`** 安装（以 580 为例；较新的 Blackwell GPU 如 RTX 50系列必须用 `-open` 变体）：
```bash
sudo apt-get install -y nvidia-driver-580-open
```

> ⚠️ **装完后重启系统**。仅 `modprobe nvidia` 通常不足以让 `nvidia-smi` 工作（实测报 *NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver*）。如果启用了 Secure Boot 还需要在重启时完成 MOK 密钥注册。

重启后验证：
```bash
nvidia-smi    # 应输出 GPU 型号 + 驱动版本
```

### 2.3 安装 nvidia-container-toolkit (GPU 直通)

官方源（如果能访问 `nvidia.github.io`）：
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

**国内用户** 如果上面请求被重置，改用 USTC 镜像：
```bash
curl -fsSL https://mirrors.ustc.edu.cn/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://mirrors.ustc.edu.cn/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://nvidia.github.io#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://mirrors.ustc.edu.cn#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

然后安装并配置 Docker 运行时：
```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
```

> ⚠️ `nvidia-ctk runtime configure` **会覆写** `/etc/docker/daemon.json`。如果之前配置过 `registry-mirrors`，要重新加回去：
>
> ```bash
> sudo tee /etc/docker/daemon.json >/dev/null <<'EOF'
> {
>     "runtimes": {
>         "nvidia": {
>             "args": [],
>             "path": "nvidia-container-runtime"
>         }
>     },
>     "registry-mirrors": [
>         "https://docker.m.daocloud.io",
>         "https://docker.1ms.run",
>         "https://dockerproxy.com"
>     ]
> }
> EOF
> ```

重启并验证：
```bash
sudo systemctl restart docker
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
# 应输出你的 GPU 型号和驱动版本
```

---

## 3. NGC 账号 (可选)

> **实测**：当前 `nvcr.io/nvidia/isaac-sim:5.1.0` **可匿名拉取**（`docker manifest inspect` 不报 401）。如果未来 NVIDIA 收紧权限或你需要拉取其他受限镜像，才需要以下步骤。`start.sh` 允许未登录下继续。

Isaac Sim 镜像托管在 NVIDIA NGC。

### 3.1 注册

1. 打开 [https://ngc.nvidia.com/](https://ngc.nvidia.com/)
2. 点击右上角 **Sign Up**，使用邮箱注册（或用 Google/GitHub 登录）
3. 完成注册后登录

### 3.2 生成 API Key

1. 登录后，点击右上角头像 → **Setup**
2. 点击 **Generate API Key**
3. **复制并保存** API Key（只显示一次）

### 3.3 Docker 登录 NGC

```bash
docker login nvcr.io --username '$oauthtoken' --password <你的API_KEY>
```

> 将 `<你的API_KEY>` 替换为上一步复制的 Key。`$oauthtoken` 是固定用户名，不要修改。

验证 (不会下载完整镜像, 只检查认证):
```bash
docker manifest inspect nvcr.io/nvidia/isaac-sim:5.1.0 >/dev/null && echo "NGC OK"
# 不应报 401 Unauthorized; 若提示需要 --experimental, 可改用:
# docker pull nvcr.io/nvidia/isaac-sim:5.1.0   # 直接开拉, 401 会立即报错
```

> **提示**: 首次拉取 Isaac Sim 镜像约 20GB，请确保网络畅通。

---

## 4. 准备场景文件

仿真需要 3D 场景文件 (USDZ 格式)。

**获取方式**: 使用 [MT Real2Sim 教程](mt-r2s/tutorial.md) 将 MindCloudX AI 平台导出的 3DGS + Mesh 数据转换为 Isaac Sim 可用的 USDZ 文件。

### 4.1 放置文件

把 `.usdz` 文件放到 `~/dataset/isaac/` 下（这是默认挂载点），目录结构可以是：

```
~/dataset/isaac/
  ├── playground02.usdz          # 单文件直接放
  └── tunnel02/                  # 或按场景分子目录
      └── usdz/scene_final.usdz
```

然后在 `r2s2r/configs/config.yaml` 中设置 `scene.environment.usdz_path` 指向对应文件，**路径必须是 `~/dataset/isaac/...` 开头**（容器内通过软链可见）。

### 4.2 ⚠️ 常见坑：`~/dataset/isaac` 被 root 占有

如果之前 `start.sh` 在 `~/dataset/isaac` 不存在时启动过，Docker 会用 root 身份自动创建这个目录，导致后续 `mv`/`cp` 报 `Permission denied`：

```bash
# 检查所有权
ls -ld ~/dataset/isaac
# 若 owner 是 root, 改回当前用户:
sudo chown -R $USER:$USER ~/dataset
```

### 4.3 路径不能写宿主机其他位置

❌ **错误示例**（容器看不到）：
```yaml
usdz_path: "/home/mt/Downloads/playground02.usdz"
```
容器内**只挂载了 `$SCENE_DIR`**（默认 `~/dataset/isaac`）到 `/scenes`，其他宿主机路径访问不到。这种配置会触发 fallback 到默认地面。

✅ **正确**：
```yaml
usdz_path: "~/dataset/isaac/playground02.usdz"
```

如果非要用别处的目录，启动时显式重映射：
```bash
./start.sh --scene ~/Downloads   # 把 ~/Downloads 当作 /scenes 挂进容器
```

### 4.4 无场景文件时的行为

如果暂时没有场景文件，仿真会打印 `WARN: USDZ 文件不存在, 跳过环境加载并回退到默认地面`，自动加载一片空白地面，机器人可正常 spawn 并被键盘控制（功能测试用足够）。

---

## 5. 一键启动

```bash
cd mt-real2sim-tutorial
./start.sh
```

脚本会自动完成:
1. ✅ 检查 Docker、GPU、NGC 登录状态
2. ✅ 启动 Isaac Sim + ROS 容器
3. ✅ 等待 Isaac Sim 就绪
4. ✅ 启动 RViz 可视化
5. ✅ 启动键盘控制

### 启动耗时（实测）

| 阶段 | 耗时 | 说明 |
|------|------|------|
| 首次镜像拉取/构建 | 10–25 min | 拉取 `nvcr.io/nvidia/isaac-sim:5.1.0` (~20GB) + 构建 ROS1 镜像 |
| **首次 Isaac Sim 冷启动** | **5–10 min** | 即使镜像已构建，首次启动仍需编译 Vulkan shader、初始化场景。实测 7 分钟以上 |
| 后续启动 | **30–60 秒** | shader 缓存到 named volume `isaac-computecache`，复用 |

> ⚠️ **强烈建议首次运行时加大超时**，否则默认 600s 很可能在 shader 还没编译完时就超时：
> ```bash
> MAX_WAIT=1800 ./start.sh
> ```
>
> 看到 `已等待 XXXs / 1800s` 持续刷新且没有错误时，**耐心等**，不要 Ctrl+C。可另开终端跑 `docker logs -f r2s2r_isaac` 观察实时进度。

### 启动后操作

1. 等待 Isaac Sim 窗口出现
2. **在 Isaac Sim 窗口中点击 ▶ PLAY 按钮**
3. 回到终端，用键盘控制机器人

> 💡 **GNOME "应用程序无响应" 弹窗**：首次冷启 shader 编译期间 Isaac Sim 主线程会阻塞，GNOME 会弹出强制退出对话框。**点击"等待"(Wait)**，编译完成后会自动恢复。如想抑制，临时调大检测间隔：
> ```bash
> gsettings set org.gnome.mutter check-alive-timeout 60000   # 60 秒
> ```

### 5.4 验证仿真与 ROS 通讯

启动成功后，**另开一个终端**做以下验证（确认 bridge / RViz 拿到了数据）：

```bash
# 进入 ROS 容器
docker exec -it r2s2r_ros1 bash -ic "rostopic list"
```

**应输出**至少以下 topics：
```
/camera/image/compressed   # 机器人前置相机 (JPEG 压缩)
/cmd_vel                   # 键盘 teleop 发出的速度指令 (输入)
/imu/data                  # 机身 IMU
/registered_scan           # LiDAR 点云
/state_estimation          # 机器人位姿
/tf                        # 坐标系树
/rosout /rosout_agg        # ROS 日志
```

**进一步排查**（任选）：
```bash
# 单 topic 频率: 看 LiDAR 是否在出
docker exec -it r2s2r_ros1 bash -ic "rostopic hz /registered_scan"
# 应看到 ~5 Hz

# 看 teleop 是否在发指令: 在 teleop 终端按 W, 这里应不断打印 vx>0
docker exec -it r2s2r_ros1 bash -ic "rostopic echo /cmd_vel"

# 节点是否都在
docker exec -it r2s2r_ros1 bash -ic "rosnode list"
# 应包含 /sim_bridge
```

如果 `rostopic list` 只有 `/rosout` 没别的，说明 bridge 没起来。看 `/tmp/r2s2r/bridge.log` 和 §10 常见问题。

---

## 6. 键盘控制

确保运行 `start.sh` 的终端拥有键盘焦点。

| 按键 | 动作 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左移 |
| D | 右移 |
| Q | 左转 |
| E | 右转 |
| R | 复位机器人（卡住时使用）|
| Space | 急停 |
| Ctrl+C | 退出并停止全部 |

> 速度指令是持续的：按 W 后机器人会一直前进，直到按 Space 或其他方向键。

---

## 7. RViz 可视化

RViz 会自动启动，显示:
- **点云** — LiDAR 扫描的 3D 环境
- **位姿箭头** — 机器人当前位置和朝向
- **相机图像** — 机器人视角画面
- **坐标轴** — 世界坐标系参考

RViz 视角操作:
- 左键拖拽 — 旋转视角
- 中键拖拽 — 平移视角
- 滚轮 — 缩放

---

## 8. 停止

方式一: 在 teleop 终端按 **Ctrl+C**，脚本会自动停止所有容器。

方式二: 手动停止:
```bash
./start.sh --stop
```

---

## 9. 指定场景目录

如果场景文件不在默认路径 (`~/dataset/isaac`)：

```bash
./start.sh --scene /path/to/your/scenes
```

---

## 10. 常见问题

### 拉取 Docker Hub 镜像超时 (i/o timeout)

现象：`docker pull` 报 `dial tcp ...: i/o timeout` 或 `failed to do request`。

原因：国内直连 Docker Hub 不稳。解决：按本文 §2.1.★ 配置 `registry-mirrors`。验证：

```bash
docker info | grep -A3 "Registry Mirrors"   # 应列出镜像加速器
docker pull hello-world
```

### 安装 nvidia-container-toolkit 时 `Connection reset by peer`

`nvidia.github.io` 在某些网络下不可达。改用 USTC 镜像 (见 §2.3)。

### `nvidia-ctk runtime configure` 后镜像加速器失效

该命令会 **完全覆写** `/etc/docker/daemon.json`，丢掉 `registry-mirrors`。重新编辑这个文件同时保留 `runtimes` 和 `registry-mirrors` 两个字段，参考 §2.3 中的示例。

### `nvidia-smi` 安装后仍然报 "couldn't communicate with the NVIDIA driver"

需要 **重启系统**。`sudo modprobe nvidia` 在多数情况下不够，尤其是刚装完 DKMS 驱动后。

### `docker compose up` 失败

- 确认 Docker 已安装并运行: `docker info`
- 确认 GPU 直通: `docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi`
- 权限问题: `sudo usermod -aG docker $USER` 后**重新登录**（或在当前 shell 执行 `newgrp docker`）

### Isaac Sim / RViz 窗口不弹出 (SSH 或 Wayland 会话)

- 本项目需要 **本地图形会话** (X11)。纯 SSH 远程、无 `$DISPLAY` 环境下 GUI 无法显示
- Ubuntu 默认 GNOME Wayland 会话：登录界面选择 "Ubuntu on Xorg"，或者在当前 Wayland 下确保 `xhost +local:docker` 能成功执行
- 验证: `echo $DISPLAY` 应为 `:0` / `:1` 之类非空值

### NGC pull 失败 / 401 Unauthorized

- 确认已执行 `docker login nvcr.io`（见第 3 节）
- API Key 可能过期，重新生成一个

### Isaac Sim 窗口不出现

- 检查 GPU 驱动: `nvidia-smi`
- 查看容器日志: `docker logs r2s2r_isaac`

### RViz 窗口不出现或黑屏

- 脚本已自动执行 `xhost +local:docker`。如果仍有问题:
  ```bash
  echo $DISPLAY    # 通常是 :0 或 :1
  xhost +local:docker
  ```
- 如果 RViz 显示 `llvmpipe` (软件渲染): 重启容器 `./start.sh --stop && ./start.sh`

### 键盘控制无反应

1. 确认 Isaac Sim 已点击 **▶ PLAY**
2. 确认运行 `start.sh` 的终端有键盘焦点
3. 如果终端显示了控制提示但机器人不动: 等待 `Isaac Sim 已就绪` 提示出现后再操作

### 启动超时

- **首次冷启** 编译 shader 通常 5–10 分钟，默认 600s 不够，请用 `MAX_WAIT=1800 ./start.sh`
- 反复超时: 查容器日志 `docker logs r2s2r_isaac`，或看 `/tmp/r2s2r/{roscore,bridge,rviz}.log`
- 看到 `FileNotFoundError: USDZ 文件不存在` 是正常的（无场景文件时自动回退默认地面）；看到 `Segmentation fault` 或 `out of memory` 则可能是显存不足

### 仿真里的机器人不动 / 没有地面

- 看 Isaac Sim 窗口日志或 `docker logs r2s2r_isaac | grep -E "scene_builder|WARN|Error"`
- 如果显示 `WARN: USDZ 文件不存在, 跳过环境加载并回退到默认地面` ——这是预期行为，地面会自动加载
- Go2 初始位置在 config.yaml 里写的是 Spot 的坐标 `[55, -2.5, 0.2]`；如需修正可改成 `[1.0, 1.0, 0.9]`（位于 `r2s2r/configs/config.yaml` 的 `quadruped.position`）

### RViz 空白 / 终端 WASD 没反应 / `rostopic list` 只有 `/rosout`

bridge 节点没起来。常见原因：

1. **`/tmp/r2s2r/bridge.log` 显示 `bash: roslaunch: command not found`**
   - 代码里有 bash 运算符优先级 bug 没修：`source ... && roscore ... &` 会把整个左半放进子 shell，导致 source 失效。
   - 当前仓库 `start.sh` 已修复（用 `;` 分隔 + 拆成两个 `docker exec`）。如果还报这个错，**说明你拿到的是旧版本代码**，对比 `start.sh` 中"启动 roscore"附近的写法。

2. **Isaac Sim 没点 PLAY**
   - bridge.launch 用了 `--wait`，会等 master，但仿真端 TCP 服务在 PLAY 之后才真正出数据。看到 RViz 空白先去点 PLAY。

3. **`/cmd_vel` 没人订阅** (`rostopic info /cmd_vel` 显示 Subscribers: None)
   - 说明 sim_bridge 没起。检查 `docker exec r2s2r_ros1 cat /tmp/bridge.log`

---

## 11. 维护者参考

### 11.1 项目结构

```
mt-real2sim-tutorial/
├── start.sh                    # 一键启动入口 (包含前置检查/容器编排/teleop 启动)
├── docker-full/                # Docker 构建上下文
│   ├── Dockerfile.isaac        # 基于 nvcr.io/nvidia/isaac-sim:5.1.0
│   ├── Dockerfile.ros1         # 基于 ros:noetic-perception
│   ├── docker-compose.yml      # 服务编排, GPU+X11 透传, named volumes
│   ├── entrypoint_isaac.sh     # 容器内启动 run_sim.py + 软链 ~/dataset/isaac
│   └── entrypoint_ros1.sh      # 容器内 source ROS, 然后 sleep infinity
├── r2s2r/                      # Isaac Sim 端 Python (bind mount 到容器 /workspace/r2s2r:ro)
│   ├── run_sim.py              # 仿真主循环, 启动 TCP server on :9090
│   ├── core/                   # 场景搭建 / 机器人工厂 / 传感器套件
│   │   ├── scene_builder.py    # USDZ 加载 (含缺失回退)
│   │   └── robot_factory.py    # Spot/Go2 工厂 + _build_common_scene
│   ├── bridge/                 # Isaac → ROS 的 TCP 协议端
│   └── configs/
│       └── config.yaml         # ⭐ 用户主要修改的文件
├── ros1_ws/                    # ROS1 包源码 (bind mount 到容器 /catkin_ws/src:ro)
│   ├── sim_bridge/             # 关键: TCP client → ROS topic 转换
│   ├── nav_demo/               # RViz 配置 (minimal.rviz)
│   ├── nav_algo/, odin_interfaces/
│   └── ros_teleop.sh           # 容器内的 teleop 脚本入口
├── lidar_configs/              # LiDAR 内参 (Isaac Sim 端读取)
├── tools/                      # 调试/数据采集小脚本
└── docs/
    └── user_guide.md           # 本文
```

### 11.2 关键文件改动指南

| 修改诉求 | 改哪里 |
|---------|--------|
| 换场景 USDZ | `r2s2r/configs/config.yaml` → `scene.environment.usdz_path` |
| 换机器人 (Spot ↔ Go2) | `r2s2r/configs/config.yaml` → `quadruped.robot_type` + 对应 `position` |
| 调键盘速度 | `r2s2r/configs/config.yaml` → `control.{linear_speed,lateral_speed,yaw_speed}` |
| 调 LiDAR 频率/精度 | `r2s2r/configs/config.yaml` → `bridge.pointcloud_rate` + `lidar_configs/` |
| 调 RViz 显示 | `ros1_ws/nav_demo/rviz/minimal.rviz` |
| 改 bridge ROS 端逻辑 | `ros1_ws/sim_bridge/`（改完无需重建镜像，bind mount） |
| 改 Isaac Sim 端逻辑 | `r2s2r/`（同上，bind mount，重启容器即可） |
| 改 Docker 镜像内置依赖 | `docker-full/Dockerfile.*` → `./start.sh --stop` 后 `docker compose build` |

### 11.3 Docker 资源占用

| 资源 | 名称 | 用途 | 大小 |
|------|------|------|------|
| Image | `r2s2r-isaac:5.1.0` | Isaac Sim 容器 | ~23 GB |
| Image | `r2s2r-ros1:noetic` | ROS1 + catkin_ws 编译产物 | ~5 GB |
| Volume | `docker-full_isaac-data` | Isaac Sim 用户数据 | <100 MB |
| Volume | `docker-full_isaac-cache` | Kit cache | ~1 GB |
| Volume | `docker-full_isaac-computecache` | ⭐ Vulkan shader 缓存（删了会重新冷启 5–10 min） | ~500 MB |
| Volume | `docker-full_isaac-logs` | 日志 | <100 MB |
| Volume | `docker-full_isaac-config` | 用户配置 | <100 MB |

### 11.4 完全清理 / 重置

```bash
./start.sh --stop                   # 停容器
docker compose -f docker-full/docker-compose.yml down -v   # 删 volumes (会触发下次冷启)
docker rmi r2s2r-isaac:5.1.0 r2s2r-ros1:noetic            # 删镜像 (会重新构建/拉取)
sudo rm -rf ~/.docker/buildx        # 可选: 清 buildx 缓存
```

### 11.5 调试常用命令

```bash
# 实时跟踪 Isaac Sim 输出
docker logs -f r2s2r_isaac
# 进入容器交互
docker exec -it r2s2r_isaac bash
docker exec -it r2s2r_ros1 bash -ic 'bash'   # bash -ic 保证 ROS 已 source
# 重启单个服务而不影响另一个
docker compose -f docker-full/docker-compose.yml restart isaac
# 查看 ROS topic 数据流
docker exec -it r2s2r_ros1 bash -ic "rostopic hz /registered_scan /imu/data /state_estimation"
```

### 11.6 已知风险点 / TODO

- **bash 运算符优先级**：`start.sh` 中 `docker exec -d bash -c "..."` 的命令拼接对 `&` / `&&` / `;` 顺序敏感，改写时要测 `which roslaunch`。
- **GUI 依赖 X11**：Wayland / 纯 SSH 环境下 Isaac Sim 窗口出不来。考虑后续加 headless 模式 + WebRTC 流。
- **Go2 初始位置坐标**仍是 Spot 的（[55, -2.5, 0.2]），需手动改 `quadruped.position` 为 `[1.0, 1.0, 0.9]`。
- **场景 fallback**：USDZ 缺失时只 print WARN 不会让用户在 host 终端看见——start.sh 启动后可考虑 `grep "WARN: USDZ"` 一次容器日志，把警告抛到主终端。
- **NGC 登录假设**：start.sh 的 NGC 检测有 fallback，但首次构建若 nvcr.io 收紧权限会失败。

---

## Maintainer

Ethan TU — [Manifoldtech](https://www.manifoldtech.cn/)