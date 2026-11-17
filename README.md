# Isaac Navigation Sim

Isaac Sim 5.1 仿真 ↔ ROS1 Noetic 统一导航算法框架。支持三类机器人，用于导航算法的仿真测试和实机部署验证。

## 特性

- **三类机器人支持**：kinematic_car（运动学履带车）、Spot、Go2
- **一键启动**：`./start.sh` 自动启动 Isaac Sim + ROS1 双容器
- **零代码切换**：通过命令行参数或配置文件切换机器人类型
- **仿真到实机**：仿真与实机使用相同的 ROS1 接口，导航算法零修改部署

## 系统架构

```
┌──────────────── Docker 容器环境 ──────────────────┐
│                                                    │
│  Isaac Sim 容器                                    │
│  ┌────────────────────────────────────┐            │
│  │ Robot (kinematic_car/Spot/Go2)     │            │
│  │ SensorSuite (LiDAR + Camera + IMU) │            │
│  │ TcpBridgeServer ───────────────────────┐        │
│  │   send: pose + pointcloud + image     │        │
│  │   recv: cmd_vel                       │        │
│  └────────────────────────────────────┘  │        │
│                                           │        │
│              TCP localhost:9090            │        │
│                                           │        │
│  ROS1 Noetic 容器                          │        │
│  ┌─────────────────────────────────────┐  │        │
│  │  sim_bridge    → /state_estimation  │◄─┘        │
│  │                → /registered_scan   │           │
│  │                → /camera/image      │           │
│  │                → TF: map → sensor   │           │
│  │                ← /cmd_vel           │           │
│  │                                     │           │
│  │  nav_algo      导航算法 (C++)       │           │
│  │  nav_demo      演示/测试 (Python)   │           │
│  │  RViz          可视化               │           │
│  └─────────────────────────────────────┘           │
└────────────────────────────────────────────────────┘

实机部署: odin_driver 替换 sim_bridge, nav_algo 零修改
```

## 快速开始

### 依赖

- **系统**：Ubuntu 22.04 / 24.04
- **GPU**：NVIDIA RTX 3070+ (显存 >= 8GB)
- **驱动**：NVIDIA >= 560
- **Docker**：docker + docker-compose-v2 + nvidia-container-toolkit
- **Isaac Sim**：已有 `isaacsim51-r2s2r:latest` Docker 镜像

### 启动

```bash
# 1. 默认启动（kinematic_car）
./start.sh

# 2. 启动 Spot 机器人
./start.sh --robot spot

# 3. 启动 Go2 机器人
./start.sh --robot go2

# 4. 指定场景目录
./start.sh --scene /path/to/dataset

# 5. 停止
./start.sh --stop
```

### 首次启动

首次启动需要：
1. 拉取 ROS Docker 镜像（约 2GB）
2. 编译 ROS 工作空间（约 1-2 分钟）
3. Isaac Sim 编译 shader（首次约 5-10 分钟）

建议首次使用大超时：
```bash
MAX_WAIT=1800 ./start.sh
```

后续启动约 30-60 秒。

## 机器人类型

| 类型 | 用途 | 特点 |
|------|------|------|
| **kinematic_car** | 导航算法验证 | 纯运动学模型，无物理仿真，快速迭代 |
| **spot** | Boston Dynamics Spot | RL 步态策略，物理仿真 |
| **go2** | Unitree Go2 | RL 步态策略，物理仿真 |

**推荐**：导航算法开发优先使用 `kinematic_car`，调试完成后再迁移到 `spot` 或 `go2`。

## ROS1 接口

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/state_estimation` | `nav_msgs/Odometry` | 机器人位姿+速度 (frame: map) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | 世界坐标系 LiDAR 点云 |
| `/camera/image` | `sensor_msgs/Image` | 相机图像 (JPEG 压缩) |
| TF: `map → base_link → sensor` | `tf/tfMessage` | 坐标变换 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 (linear.x/y + angular.z) |

## 项目结构

```
isaac_navigation_sim/
├── r2s2r/                       # Isaac Sim 仿真核心
│   ├── core/                    #   机器人、传感器、场景
│   │   ├── kinematic_vehicle.py #     运动学车辆
│   │   ├── robot_factory.py     #     机器人工厂
│   │   ├── sensor_suite.py      #     传感器套件
│   │   └── ...
│   ├── bridge/                  #   TCP 桥接 (无 ROS 依赖)
│   ├── configs/                 #   配置 + 标定
│   └── run_sim.py               #   主入口
│
├── ros1_ws/src/                 # ROS1 工作空间
│   ├── sim_bridge/              #   TCP↔ROS1 桥接
│   ├── nav_algo/                #   导航算法 (C++)
│   ├── nav_demo/                #   演示/测试 (Python)
│   └── odin_interfaces/         #   真机接口占位
│
├── docker/                      # Docker 配置
│   ├── docker-compose.yml       #   双容器编排
│   └── Dockerfile.ros1          #   ROS1 镜像
│
├── tools/                       # 工具脚本
└── start.sh                     # 一键启动脚本
```

## 配置文件

主配置文件：`r2s2r/configs/config.yaml`

```yaml
# 机器人类型（也可通过 start.sh --robot 覆盖）
quadruped:
  robot_type: "kinematic_car"  # kinematic_car | spot | go2

# 场景
scene:
  environment:
    usdz_path: "/dataset/tunnel1.usdz"

# 传感器标定（自动根据 robot_type 适配）
sensor_suite:
  calibration_dir: "calibration/kinematic_car"  # or pinhole, odin1
```

## 实机部署

仿真中验证的导航算法可直接部署到实机：

```bash
# 实机上（Ubuntu 20.04 + ROS Noetic）
roslaunch odin_interfaces driver.launch  # 替换 sim_bridge
roslaunch nav_algo navigation.launch     # 相同的导航算法
```

ROS 接口完全兼容，无需修改代码。

## 故障排查

### 容器启动失败

```bash
# 检查 Docker
docker info
groups | grep docker

# 检查 GPU
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# 检查镜像
docker images | grep isaac
```

### Isaac Sim 启动超时

首次启动需要编译 shader（5-10 分钟）：
```bash
MAX_WAIT=1800 ./start.sh
```

查看 Isaac Sim 日志：
```bash
docker logs isaac_nav_isaac
```

### ROS topics 无数据

检查 bridge 日志：
```bash
docker exec isaac_nav_ros1 cat /tmp/bridge.log
```

手动测试 bridge：
```bash
docker exec -it isaac_nav_ros1 bash
source /catkin_ws/devel/setup.bash
rostopic list
rostopic hz /registered_scan
```

## 开发

### 添加新的导航算法

1. 在 `ros1_ws/src/` 创建新的 ROS 包
2. 订阅 `/state_estimation` 和 `/registered_scan`
3. 发布 `/cmd_vel`
4. 在仿真中测试

### 修改传感器配置

编辑 `r2s2r/configs/calibration/<robot_type>/` 下的标定文件。

### 调试

```bash
# 进入 Isaac Sim 容器
docker exec -it isaac_nav_isaac bash

# 进入 ROS 容器
docker exec -it isaac_nav_ros1 bash
source /catkin_ws/devel/setup.bash
```

## 许可

基于 tunnel-inspection-v2 和 MT-Real2Sim-Tutorial 融合。


## Simulation updates

Go2 MoE-CTS + PI, ODIN1 calibration, synchronized lidar/ROS simulation time,
and diagnostics are described in [docs/simulation_updates.md](docs/simulation_updates.md).
