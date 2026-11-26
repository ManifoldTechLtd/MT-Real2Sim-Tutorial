English | [简体中文](README.zh-CN.md)

# Isaac Navigation Sim

An Isaac Sim 5.1 simulation ↔ ROS1 Noetic unified navigation algorithm framework. It supports three robot types for simulation testing and real-robot deployment validation of navigation algorithms.

## Features

- **Three robot types**: kinematic_car (kinematic tracked vehicle), Spot, Go2
- **One-command launch**: `./start.sh` starts the Isaac Sim + ROS1 dual-container stack automatically
- **Zero-code switching**: switch robot types via command-line flags or configuration files
- **Simulation to real robot**: simulation and real robot use the same ROS1 interface, so navigation algorithms deploy without modification

## System architecture

```
┌──────────────── Docker containers ─────────────────┐
│                                                    │
│ Isaac Sim container                                │
│  ┌────────────────────────────────────┐            │
│  │ Robot (kinematic_car/Spot/Go2)     │            │
│  │ SensorSuite (LiDAR + Camera + IMU) │            │
│  │ TcpBridgeServer ──────────────────────┐         │
│  │   send: pose + pointcloud + image  │  │         │
│  │   recv: cmd_vel                    │  │         │
│  └────────────────────────────────────┘  │         │
│                                          │         │
│                  TCP localhost:9090                │
│                                          │         │
│ ROS1 Noetic container                              │
│  ┌────────────────────────────────────┐  │         │
│  │  sim_bridge    → /state_estimation │◄─┘         │
│  │                → /registered_scan  │            │
│  │                → /camera/image     │            │
│  │                → TF: map → sensor  │            │
│  │                ← /cmd_vel          │            │
│  │                                    │            │
│  │  nav_algo      navigation (C++)    │            │
│  │  nav_demo      demo/tests (Python) │            │
│  │  RViz          visualization       │            │
│  └────────────────────────────────────┘            │
└────────────────────────────────────────────────────┘

Real robot: odin_driver replaces sim_bridge; nav_algo unchanged
```

## Quick start

### Dependencies

- **OS**: Ubuntu 22.04 / 24.04
- **GPU**: NVIDIA RTX 3070+ (>= 8 GB VRAM)
- **Driver**: NVIDIA >= 560
- **Docker**: docker + docker-compose-v2 + nvidia-container-toolkit
- **Isaac Sim**: `r2s2r-isaac:5.1.0` Docker image (can be built/renamed from `nvcr.io/nvidia/isaac-sim:5.1.0`, see `docker/Dockerfile.isaac`)

### Launch

```bash
# 1. Default launch (kinematic_car)
./start.sh

# 2. Launch the Spot robot
./start.sh --robot spot

# 3. Launch the Go2 robot
./start.sh --robot go2

# 4. Specify the scene directory
./start.sh --scene /path/to/dataset

# 5. Stop
./start.sh --stop
```

### First launch

The first launch requires:
1. Pulling the ROS Docker image (about 2 GB)
2. Building the ROS workspace (about 1-2 minutes)
3. Isaac Sim compiling shaders (about 5-10 minutes the first time)

Using a large timeout for the first run is recommended:
```bash
MAX_WAIT=1800 ./start.sh
```

Subsequent launches take about 30-60 seconds.

## ONS2 navigation integration

The simulator can run the ONS2 navigation stack directly through `sim_bridge` (ROS1) and `sim_bridge_ros2` (ROS2);
topics and TF contracts are identical to the real robot. **The single source of truth for usage documentation** is in the ONS2 repository:
[04 · Simulation](../Odin-Nav-Stack2/docs/simulation/04_simulation.zh-CN.md).

```bash
./start.sh --robot go2 --runtime ros2 --stack ons2 --map office_0923_sim
```

Integration assets (maps, waypoints, smoke tooling) are in [`integrations/ons2/`](integrations/ons2/README.md);
topic/TF contracts are in [`docs/simulation-interface.md`](docs/simulation-interface.md).
The full flow from Q9000 scanning and the XAI export through USDZ/map generation to ONS2 navigation is described in
[`docs/Use the ONS2 and simulator manual.md`](docs/Use%20the%20ONS2%20and%20simulator%20manual.md).

ONS2 also requires: the Go2 simulation profile/launch from the navigation repository, the USDZ/PGM/waypoint assets exported per the tutorial,
and a locally built `isaac_nav_ros2` image. See the navigation repository simulation guide for the profile and startup details.

## Robot types

| Type | Purpose | Characteristics |
|------|---------|-----------------|
| **kinematic_car** | Navigation algorithm validation | Pure kinematic model, no physics simulation, fast iteration |
| **spot** | Boston Dynamics Spot | RL gait policy, physics simulation |
| **go2** | Unitree Go2 | RL gait policy, physics simulation |

**Recommendation**: for navigation algorithm development, prefer `kinematic_car`, and migrate to `spot` or `go2` after debugging is complete.

## ROS1 interface

| Topic | Message type | Description |
|-------|--------------|-------------|
| `/state_estimation` | `nav_msgs/Odometry` | Robot pose + velocity (frame: map) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | LiDAR point cloud in the world frame |
| `/camera/image` | `sensor_msgs/Image` | Camera image (JPEG compressed) |
| TF: `map → base_link → sensor` | `tf/tfMessage` | Coordinate transforms |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command (linear.x/y + angular.z) |

## Project structure

```
isaac_navigation_sim/
├── r2s2r/                       # Isaac Sim simulation core
│   ├── core/                    #   robots, sensors, scene
│   │   ├── kinematic_vehicle.py #     kinematic vehicle
│   │   ├── robot_factory.py     #     robot factory
│   │   ├── sensor_suite.py      #     sensor suite
│   │   └── ...
│   ├── bridge/                  #   TCP bridge (no ROS dependency)
│   ├── configs/                 #   configurations + calibration
│   └── run_sim.py               #   main entry point
│
├── ros1_ws/src/                 # ROS1 workspace
│   ├── sim_bridge/              #   TCP↔ROS1 bridge
│   ├── nav_algo/                #   navigation algorithms (C++)
│   ├── nav_demo/                #   demo/tests (Python)
│   └── odin_interfaces/         #   real-robot interface placeholder
│
├── docker/                      # Docker configuration
│   ├── docker-compose.yml       #   dual-container orchestration
│   └── Dockerfile.ros1          #   ROS1 image
│
├── tools/                       # utility scripts
└── start.sh                     # one-command launch script
```

## Configuration files

Main configuration file: `r2s2r/configs/config.yaml`

```yaml
# Robot type (can also be overridden by start.sh --robot)
quadruped:
  robot_type: "kinematic_car"  # kinematic_car | spot | go2

# Scene
scene:
  environment:
    usdz_path: "/dataset/tunnel1.usdz"

# Sensor calibration (adapted automatically according to robot_type)
sensor_suite:
  calibration_dir: "calibration/kinematic_car"  # or pinhole, odin1
```

## Real-robot deployment

Navigation algorithms validated in simulation can be deployed directly to the real robot:

```bash
# On the real robot (Ubuntu 20.04 + ROS Noetic)
roslaunch odin_interfaces driver.launch  # replaces sim_bridge
roslaunch nav_algo navigation.launch     # the same navigation algorithms
```

The ROS interface is fully compatible; no code changes are needed.

## Troubleshooting

### Container launch failure

```bash
# Check Docker
docker info
groups | grep docker

# Check the GPU
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# Check the images
docker images | grep isaac
```

### Isaac Sim startup timeout

The first launch needs to compile shaders (5-10 minutes):
```bash
MAX_WAIT=1800 ./start.sh
```

View the Isaac Sim logs:
```bash
docker logs isaac_nav_isaac
```

### No data on ROS topics

Check the bridge log:
```bash
docker exec isaac_nav_ros1 cat /tmp/bridge.log
```

Test the bridge manually:
```bash
docker exec -it isaac_nav_ros1 bash
source /catkin_ws/devel/setup.bash
rostopic list
rostopic hz /registered_scan
```

## Development

### Adding a new navigation algorithm

1. Create a new ROS package under `ros1_ws/src/`
2. Subscribe to `/state_estimation` and `/registered_scan`
3. Publish `/cmd_vel`
4. Test in simulation

### Modifying sensor configuration

Edit the calibration files under `r2s2r/configs/calibration/<robot_type>/`.

### Debugging

```bash
# Enter the Isaac Sim container
docker exec -it isaac_nav_isaac bash

# Enter the ROS container
docker exec -it isaac_nav_ros1 bash
source /catkin_ws/devel/setup.bash
```

## License

Based on a fusion of tunnel-inspection-v2 and MT-Real2Sim-Tutorial.


## Simulation updates

Go2 MoE-CTS + PI, ODIN1 calibration, synchronized lidar/ROS simulation time,
and diagnostics are described in [docs/simulation_updates.md](docs/simulation_updates.md).
