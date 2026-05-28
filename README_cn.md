# MT Real2Sim Tutorial

[English](README.md)

Isaac Sim 5.1 仿真 ↔ ROS1 Noetic 真机统一算法框架。仿真与真机使用相同的 ROS1 接口和算法代码，零修改部署。

## 系统架构

```
┌──────────────── 宿主机 (Linux + NVIDIA 驱动) ──────────────────┐
│                                                                │
│  Docker: Isaac Sim 5.1                                         │
│  ┌────────────────────────────────────────┐                    │
│  │ RL Policy (Go2)                         │                    │
│  │ SensorSuite (RTX LiDAR + Camera + IMU)  │                    │
│  │ TcpBridgeServer ───────────────────────────┐                │
│  │   send: pose + pointcloud + image          │                │
│  │   recv: cmd_vel                            │                │
│  └────────────────────────────────────────┘   │                │
│                                                │                │
│              TCP localhost:9090                 │                │
│                                                │                │
│  Docker: ROS Noetic                            │                │
│  ┌─────────────────────────────────────────┐   │                │
│  │  sim_bridge    → /state_estimation      │◄──┘                │
│  │                → /registered_scan       │                    │
│  │                → /camera/image          │                    │
│  │                → TF: map → sensor       │                    │
│  │                ← /cmd_vel               │                    │
│  │                                         │                    │
│  │  nav_algo (C++)  terrain_analysis       │                    │
│  │                  local_planner          │                    │
│  │                  path_follower          │                    │
│  │                                         │                    │
│  │  nav_demo (Py)   teleop / auto_nav     │                    │
│  │  RViz                                   │                    │
│  └─────────────────────────────────────────┘                    │
└────────────────────────────────────────────────────────────────┘

真机部署: odin_driver 替换 sim_bridge, nav_algo + nav_demo 零修改
```

## 项目结构

```
mt-real2sim-tutorial/
├── r2s2r/                             # Isaac Sim 仿真侧 Python 包
│   ├── core/                          #   仿真核心模块
│   │   ├── sensor_suite.py            #     Camera + LiDAR + IMU 统一管理
│   │   ├── scene_builder.py           #     USDZ 环境导入 + 碰撞 + 光照
│   │   ├── robot_factory.py           #     机器人工厂 (Go2)
│   │   ├── go2_policy.py              #     Go2 RL 策略控制器
│   │   └── perf_timer.py              #     性能计时器
│   ├── bridge/                        #   TCP 桥接 (无 ROS 依赖)
│   │   ├── protocol.py                #     二进制协议 (magic + msgpack)
│   │   └── tcp_server.py              #     非阻塞 TCP 服务端
│   ├── configs/                       #   仿真配置 + 传感器标定
│   │   ├── config.yaml                #     统一仿真配置
│   │   ├── go2.yaml                   #     Go2 独立配置
│   │   └── calibration/pinhole/       #     Camera + LiDAR + IMU 内外参
│   └── run_sim.py                     #   主入口
│
├── ros1_ws/src/                       # ROS1 工作空间 (Docker 内 catkin)
│   ├── sim_bridge/                    #   TCP↔ROS1 桥接 (Python)
│   ├── nav_algo/                      #   核心导航算法 (C++)
│   ├── nav_demo/                      #   Demo: 遥控 + 可视化
│   └── odin_interfaces/               #   真机驱动接口 (placeholder)
│
├── docker-full/                       # Docker 双容器编排
│   ├── Dockerfile.isaac               #   Isaac Sim 镜像
│   ├── docker-compose.yml             #   Isaac Sim + ROS1 编排
│   └── entrypoint_isaac.sh            #   容器入口脚本
│
├── lidar_configs/                     # 自定义 LiDAR 配置 (详见 docs/lidar_guide.md)
│   ├── json/MT_MLXS.json             #   MT-MLXS 配置 (基于 ZVISION ML-Xs)
│   └── usd/MT_MLXS.usd              #   转换后的 USD 文件
│
├── tools/                             # 工具脚本
│   ├── convert_lidar_json.py          #   LiDAR JSON → USD 转换
│   ├── ros_teleop.sh                  #   一键 bridge + teleop
│   └── run_isaac_docker.sh            #   容器内运行 Isaac Sim
│
├── start.sh                           # 一键启动/停止脚本
│
└── docs/                              # 文档
    ├── user_guide.md                  #   用户指南 (环境配置 + 操作)
    └── lidar_guide.md                 #   自定义 LiDAR 配置指南
```

## ROS1 接口

| Topic | 消息类型 | 说明 |
|-------|---------|------|
| `/state_estimation` | `nav_msgs/Odometry` | 机器人位姿+速度 (frame: map) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | 世界坐标系 LiDAR 点云 |
| TF: `map → sensor` | `tf/tfMessage` | 传感器坐标变换 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 (linear.x/y + angular.z) |

## 快速开始

依赖: NVIDIA 驱动 (>=560) + Docker + nvidia-container-toolkit + [NGC 账号](https://ngc.nvidia.com/)

```bash
# 首次: NGC 登录
docker login nvcr.io --username '$oauthtoken' --password <YOUR_NGC_API_KEY>

# 克隆项目
git clone https://git.manifoldtech.cn/Ethan/mt-real2sim-tutorial.git
cd mt-real2sim-tutorial

# 一键启动
./start.sh

# 停止
./start.sh --stop
```

详细步骤见 [用户指南](docs/user_guide_cn.md)。
