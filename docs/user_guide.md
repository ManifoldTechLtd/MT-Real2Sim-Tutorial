# User Guide — Isaac Real2Sim2Real Go2

[简体中文](user_guide_cn.md)

Quadruped robot simulation platform. One-click launch of Isaac Sim + ROS + RViz to control a Go2 robot walking in 3D scenes via keyboard.

---

## 0. Fresh Machine Deployment Checklist (TL;DR)

> A quick checklist for "I just got the code, let me deploy" scenarios. Each item links to the relevant section with detailed steps and troubleshooting.

| # | Task | Section | Success Criterion |
|---|------|---------|-------------------|
| 1 | Install Docker + add user to docker group + **re-login** | §2.1 | `groups` contains `docker` |
| 2 | (China mainland) Configure Docker registry mirrors | §2.1.★ | `docker pull hello-world` doesn't timeout |
| 3 | Install NVIDIA driver + **reboot** | §2.2 | `nvidia-smi` shows GPU info |
| 4 | Install nvidia-container-toolkit + merge daemon.json | §2.3 | `docker run --gpus all nvidia/cuda:... nvidia-smi` works |
| 5 | (Optional) NGC login | §3 | `docker manifest inspect nvcr.io/nvidia/isaac-sim:5.1.0` no 401 |
| 6 | Place scene files in `~/dataset/isaac/`, verify ownership | §4 | `ls ~/dataset/isaac/*.usdz` has files |
| 7 | Set `usdz_path` in `r2s2r/configs/config.yaml` | §4 | Path starts with `~/dataset/isaac/` |
| 8 | First launch with large timeout | §5 | `MAX_WAIT=1800 ./start.sh` |
| 9 | Verify ROS communication after launch | §5.4 | `rostopic list` includes `/cmd_vel /registered_scan` etc. |

> ⏱️ **Time estimate**: After environment is ready, first launch ≈ **30–50 min** (20GB image pull + shader compilation); subsequent launches ≈ **1 min**.

---

## 1. System Requirements

| Item | Requirement |
|------|-------------|
| OS | Linux x86_64 (Ubuntu 22.04 / 24.04) |
| GPU | NVIDIA RTX 3070+ (VRAM >= 8GB), recommended RTX 4080+ |
| Driver | NVIDIA >= 560 (recommended 580+); RTX 50-series (Blackwell) requires `-open` variant |
| RAM | >= 32GB |
| Disk | >= 30GB free space |

> ⚠️ **VRAM note**: 8GB VRAM (e.g., RTX 5060 Mobile / 4060 / 3060) is the minimum for Isaac Sim 5.1. Close browsers and other GPU-heavy apps during runtime to avoid OOM.

---

## 2. Install Docker

### 2.1 Install Docker

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-v2

# Allow current user to run without sudo
sudo usermod -aG docker $USER
```

> ⚠️ After adding to docker group, you **must log out and back in** (or run `newgrp docker` in the current shell). Verify with `groups` that the output contains `docker`.

Verify:
```bash
docker --version
docker compose version
docker info | grep "Server Version"   # Getting output means no sudo needed
```

### 2.1.★ Registry Mirrors (required for China mainland users)

The project depends on `ros:noetic-perception` from Docker Hub. Direct access to `registry-1.docker.io` from China almost always times out — configure a registry mirror.

Create or edit `/etc/docker/daemon.json` (note: nvidia-container-toolkit installed later may overwrite this file; you can add `registry-mirrors` after that step):

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

Verify pull:
```bash
docker pull hello-world && docker rmi hello-world
```

### 2.2 Install NVIDIA Driver

Query recommended drivers:
```bash
ubuntu-drivers devices
```

Install the **`nvidia-driver-<version>-open`** variant (e.g., 580; newer Blackwell GPUs like RTX 50-series require the `-open` variant):
```bash
sudo apt-get install -y nvidia-driver-580-open
```

> ⚠️ **Reboot after installation**. Just running `modprobe nvidia` is usually insufficient for `nvidia-smi` to work (commonly reports *NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver*). If Secure Boot is enabled, you also need to complete MOK key enrollment during reboot.

Verify after reboot:
```bash
nvidia-smi    # Should show GPU model + driver version
```

### 2.3 Install nvidia-container-toolkit (GPU Passthrough)

Official source (if `nvidia.github.io` is accessible):
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

**China users** — if the above gets connection reset, use USTC mirror:
```bash
curl -fsSL https://mirrors.ustc.edu.cn/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://mirrors.ustc.edu.cn/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://nvidia.github.io#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://mirrors.ustc.edu.cn#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Then install and configure Docker runtime:
```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
```

> ⚠️ `nvidia-ctk runtime configure` **overwrites** `/etc/docker/daemon.json`. If you previously configured `registry-mirrors`, add it back:
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

Restart and verify:
```bash
sudo systemctl restart docker
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
# Should show your GPU model and driver version
```

---

## 3. NGC Account (Optional)

> **Tested**: Currently `nvcr.io/nvidia/isaac-sim:5.1.0` **can be pulled anonymously** (`docker manifest inspect` doesn't return 401). The following steps are only needed if NVIDIA tightens permissions or you need other restricted images. `start.sh` allows continuing without NGC login.

The Isaac Sim image is hosted on NVIDIA NGC.

### 3.1 Register

1. Go to [https://ngc.nvidia.com/](https://ngc.nvidia.com/)
2. Click **Sign Up** in the top-right corner (email, Google, or GitHub)
3. Log in after registration

### 3.2 Generate API Key

1. After login, click avatar → **Setup**
2. Click **Generate API Key**
3. **Copy and save** the key (shown only once)

### 3.3 Docker Login to NGC

```bash
docker login nvcr.io --username '$oauthtoken' --password <YOUR_API_KEY>
```

> Replace `<YOUR_API_KEY>` with the key from the previous step. `$oauthtoken` is the fixed username — do not change it.

Verify (does not download the full image, only checks auth):
```bash
docker manifest inspect nvcr.io/nvidia/isaac-sim:5.1.0 >/dev/null && echo "NGC OK"
# Should not return 401 Unauthorized
```

> **Note**: First pull of the Isaac Sim image is ~20GB. Ensure stable network connectivity.

---

## 4. Prepare Scene Files

The simulation requires 3D scene files (USDZ format).

**How to obtain**: Use the [MT Real2Sim Tutorial](mt-r2s/tutorial.md) to convert MindCloudX AI platform exports (3DGS + Mesh) into Isaac Sim-compatible USDZ files.

### 4.1 Place Files

Put `.usdz` files under `~/dataset/isaac/` (the default mount point). Directory structure can be:

```
~/dataset/isaac/
  ├── playground02.usdz          # Single file directly
  └── tunnel02/                  # Or organized in subdirectories
      └── usdz/scene_final.usdz
```

Then set `scene.environment.usdz_path` in `r2s2r/configs/config.yaml` to point to the file. **The path must start with `~/dataset/isaac/...`** (visible inside the container via symlink).

### 4.2 ⚠️ Common Pitfall: `~/dataset/isaac` Owned by root

If `start.sh` was run before `~/dataset/isaac` existed, Docker auto-creates the directory as root, causing `Permission denied` on subsequent `mv`/`cp`:

```bash
# Check ownership
ls -ld ~/dataset/isaac
# If owned by root, reclaim:
sudo chown -R $USER:$USER ~/dataset
```

### 4.3 Paths Must Point Inside the Mounted Directory

❌ **Wrong** (container can't see it):
```yaml
usdz_path: "/home/user/Downloads/playground02.usdz"
```
The container **only mounts `$SCENE_DIR`** (default `~/dataset/isaac`) to `/scenes`. Other host paths are inaccessible and will trigger fallback to default ground.

✅ **Correct**:
```yaml
usdz_path: "~/dataset/isaac/playground02.usdz"
```

To use a different directory, explicitly remap at launch:
```bash
./start.sh --scene ~/Downloads   # Mounts ~/Downloads as /scenes inside container
```

### 4.4 Behavior Without Scene Files

If no scene file is available, the simulation prints `WARN: USDZ file not found, skipping environment and falling back to default ground`, auto-loads a blank ground plane, and the robot can still spawn and be controlled via keyboard (sufficient for functional testing).

---

## 5. One-Click Launch

```bash
cd mt-real2sim-tutorial
./start.sh
```

The script automatically:
1. ✅ Checks Docker, GPU, and NGC login status
2. ✅ Starts Isaac Sim + ROS containers
3. ✅ Waits for Isaac Sim to be ready
4. ✅ Launches RViz visualization
5. ✅ Starts keyboard control

### Startup Timing (Real-World Measurements)

| Phase | Duration | Notes |
|-------|----------|-------|
| First image pull/build | 10–25 min | Pulling `nvcr.io/nvidia/isaac-sim:5.1.0` (~20GB) + building ROS1 image |
| **First Isaac Sim cold start** | **5–10 min** | Even after image is built, first launch compiles Vulkan shaders. Measured 7+ min |
| Subsequent launches | **30–60 sec** | Shader cache persists in named volume `isaac-computecache` |

> ⚠️ **Strongly recommended to increase timeout on first run**, otherwise the default 600s may expire before shader compilation finishes:
> ```bash
> MAX_WAIT=1800 ./start.sh
> ```
>
> If you see `waited XXXs / 1800s` continuously refreshing with no errors, **be patient** — don't Ctrl+C. Open another terminal and run `docker logs -f r2s2r_isaac` to monitor progress.

### Post-Launch Steps

1. Wait for the Isaac Sim window to appear
2. **Click the ▶ PLAY button in the Isaac Sim window**
3. Return to terminal and control the robot with keyboard

> 💡 **GNOME "Application Not Responding" dialog**: During first cold start shader compilation, the Isaac Sim main thread blocks. GNOME may show a force-quit dialog. **Click "Wait"** — it will recover once compilation completes. To suppress, temporarily increase the detection interval:
> ```bash
> gsettings set org.gnome.mutter check-alive-timeout 60000   # 60 seconds
> ```

### 5.4 Verify Simulation ↔ ROS Communication

After successful launch, **open another terminal** to verify bridge/RViz are receiving data:

```bash
# Enter the ROS container
docker exec -it r2s2r_ros1 bash -ic "rostopic list"
```

**Expected output** (at minimum):
```
/camera/image/compressed   # Front camera (JPEG compressed)
/cmd_vel                   # Keyboard teleop velocity commands (input)
/imu/data                  # Body IMU
/registered_scan           # LiDAR point cloud
/state_estimation          # Robot pose
/tf                        # TF tree
/rosout /rosout_agg        # ROS logging
```

**Further debugging** (optional):
```bash
# Check single topic frequency
docker exec -it r2s2r_ros1 bash -ic "rostopic hz /registered_scan"
# Should show ~5 Hz

# Check if teleop is sending commands: press W in teleop terminal, should print vx>0
docker exec -it r2s2r_ros1 bash -ic "rostopic echo /cmd_vel"

# Check all nodes
docker exec -it r2s2r_ros1 bash -ic "rosnode list"
# Should include /sim_bridge
```

If `rostopic list` only shows `/rosout`, the bridge didn't start. Check `/tmp/r2s2r/bridge.log` and §10 Troubleshooting.

---

## 6. Keyboard Controls

Ensure the terminal running `start.sh` has keyboard focus.

| Key | Action |
|-----|--------|
| W | Forward |
| S | Backward |
| A | Strafe left |
| D | Strafe right |
| Q | Turn left |
| E | Turn right |
| R | Reset robot (use when stuck) |
| Space | Emergency stop |
| Ctrl+C | Quit and stop all |

> Velocity commands are persistent: after pressing W the robot keeps moving forward until you press Space or another direction key.

---

## 7. RViz Visualization

RViz launches automatically and displays:
- **Point cloud** — LiDAR scan of the 3D environment
- **Pose arrow** — Robot's current position and heading
- **Camera image** — Robot's perspective view
- **Axes** — World coordinate frame reference

RViz view controls:
- Left-click drag — Rotate view
- Middle-click drag — Pan view
- Scroll wheel — Zoom

---

## 8. Stopping

Option 1: Press **Ctrl+C** in the teleop terminal — the script automatically stops all containers.

Option 2: Manual stop:
```bash
./start.sh --stop
```

---

## 9. Custom Scene Directory

If scene files are not in the default path (`~/dataset/isaac`):

```bash
./start.sh --scene /path/to/your/scenes
```

---

## 10. Troubleshooting

### Docker Hub Image Pull Timeout (i/o timeout)

Symptom: `docker pull` reports `dial tcp ...: i/o timeout` or `failed to do request`.

Cause: Unreliable direct connection to Docker Hub (common in China). Solution: configure `registry-mirrors` per §2.1.★. Verify:

```bash
docker info | grep -A3 "Registry Mirrors"   # Should list mirror URLs
docker pull hello-world
```

### `Connection reset by peer` When Installing nvidia-container-toolkit

`nvidia.github.io` is unreachable on some networks. Use USTC mirror instead (see §2.3).

### Registry Mirrors Lost After `nvidia-ctk runtime configure`

This command **completely overwrites** `/etc/docker/daemon.json`, losing `registry-mirrors`. Re-edit the file to include both `runtimes` and `registry-mirrors` fields — see the example in §2.3.

### `nvidia-smi` Still Reports "couldn't communicate with the NVIDIA driver" After Install

You need to **reboot the system**. `sudo modprobe nvidia` is insufficient in most cases, especially after a fresh DKMS driver install.

### `docker compose up` Fails

- Confirm Docker is installed and running: `docker info`
- Confirm GPU passthrough: `docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi`
- Permission issue: `sudo usermod -aG docker $USER` then **re-login** (or `newgrp docker` in current shell)

### Isaac Sim / RViz Window Doesn't Appear (SSH or Wayland Session)

- This project requires a **local graphical session** (X11). Pure SSH without `$DISPLAY` cannot show GUI
- Ubuntu defaults to GNOME Wayland: at the login screen choose "Ubuntu on Xorg", or ensure `xhost +local:docker` succeeds under Wayland
- Verify: `echo $DISPLAY` should be `:0` / `:1` or similar (non-empty)

### NGC Pull Failed / 401 Unauthorized

- Confirm you ran `docker login nvcr.io` (see §3)
- API Key may have expired — generate a new one

### Isaac Sim Window Doesn't Appear

- Check GPU driver: `nvidia-smi`
- Check container logs: `docker logs r2s2r_isaac`

### RViz Window Doesn't Appear or Shows Black Screen

- The script auto-runs `xhost +local:docker`. If still failing:
  ```bash
  echo $DISPLAY    # Usually :0 or :1
  xhost +local:docker
  ```
- If RViz shows `llvmpipe` (software rendering): restart containers `./start.sh --stop && ./start.sh`

### Keyboard Control Not Responding

1. Confirm Isaac Sim has **▶ PLAY** clicked
2. Confirm the terminal running `start.sh` has keyboard focus
3. If control prompts are displayed but robot doesn't move: wait until `Isaac Sim ready` message appears

### Startup Timeout

- **First cold start** shader compilation typically takes 5–10 min; default 600s may not be enough — use `MAX_WAIT=1800 ./start.sh`
- Repeated timeouts: check container logs `docker logs r2s2r_isaac`, or look at `/tmp/r2s2r/{roscore,bridge,rviz}.log`
- Seeing `FileNotFoundError: USDZ file not found` is normal (falls back to default ground without scene files); seeing `Segmentation fault` or `out of memory` likely means insufficient VRAM

### Robot Doesn't Move / No Ground in Simulation

- Check Isaac Sim window logs or `docker logs r2s2r_isaac | grep -E "scene_builder|WARN|Error"`
- If showing `WARN: USDZ file not found, skipping environment and falling back to default ground` — this is expected behavior, ground plane auto-loads
- Go2 initial position in config.yaml uses Spot's coordinates `[55, -2.5, 0.2]`; to fix, change to `[1.0, 1.0, 0.9]` (in `r2s2r/configs/config.yaml` → `quadruped.position`)

### RViz Blank / WASD Not Working / `rostopic list` Only Shows `/rosout`

The bridge node didn't start. Common causes:

1. **`/tmp/r2s2r/bridge.log` shows `bash: roslaunch: command not found`**
   - A bash operator precedence bug: `source ... && roscore ... &` puts the entire left side in a subshell, causing source to not take effect.
   - Current repo `start.sh` is fixed (uses `;` separator + separate `docker exec` calls). If you still see this error, **you have an old version of the code** — compare the "start roscore" section in `start.sh`.

2. **Isaac Sim PLAY not clicked**
   - bridge.launch uses `--wait` to wait for master, but the simulation TCP server only produces data after PLAY. If RViz is blank, click PLAY first.

3. **`/cmd_vel` has no subscribers** (`rostopic info /cmd_vel` shows Subscribers: None)
   - Means sim_bridge didn't start. Check `docker exec r2s2r_ros1 cat /tmp/bridge.log`

---

## 11. Maintainer Reference

### 11.1 Project Structure

```
mt-real2sim-tutorial/
├── start.sh                    # One-click entry (pre-checks / container orchestration / teleop)
├── docker-full/                # Docker build context
│   ├── Dockerfile.isaac        # Based on nvcr.io/nvidia/isaac-sim:5.1.0
│   ├── Dockerfile.ros1         # Based on ros:noetic-perception
│   ├── docker-compose.yml      # Service orchestration, GPU+X11 passthrough, named volumes
│   ├── entrypoint_isaac.sh     # Container start: run_sim.py + symlink ~/dataset/isaac
│   └── entrypoint_ros1.sh      # Container start: source ROS, then sleep infinity
├── r2s2r/                      # Isaac Sim side Python (bind mount to /workspace/r2s2r:ro)
│   ├── run_sim.py              # Main simulation loop, starts TCP server on :9090
│   ├── core/                   # Scene building / robot factory / sensor suite
│   │   ├── scene_builder.py    # USDZ loading (with missing-file fallback)
│   │   └── robot_factory.py    # Spot/Go2 factory + _build_common_scene
│   ├── bridge/                 # Isaac → ROS TCP protocol
│   └── configs/
│       └── config.yaml         # ⭐ Primary user-facing config file
├── ros1_ws/                    # ROS1 package source (bind mount to /catkin_ws/src:ro)
│   ├── sim_bridge/             # Key: TCP client → ROS topic conversion
│   ├── nav_demo/               # RViz config (minimal.rviz)
│   ├── nav_algo/, odin_interfaces/
│   └── ros_teleop.sh           # In-container teleop script entry
├── lidar_configs/              # LiDAR intrinsics (read by Isaac Sim side)
├── tools/                      # Debug/data-collection scripts
└── docs/
    └── user_guide.md           # This document
```

### 11.2 Configuration Change Guide

| Goal | Where to Change |
|------|-----------------|
| Change scene USDZ | `r2s2r/configs/config.yaml` → `scene.environment.usdz_path` |
| Switch robot (Spot ↔ Go2) | `r2s2r/configs/config.yaml` → `quadruped.robot_type` + corresponding `position` |
| Adjust keyboard speed | `r2s2r/configs/config.yaml` → `control.{linear_speed,lateral_speed,yaw_speed}` |
| Adjust LiDAR rate/precision | `r2s2r/configs/config.yaml` → `bridge.pointcloud_rate` + `lidar_configs/` |
| Adjust RViz display | `ros1_ws/nav_demo/rviz/minimal.rviz` |
| Modify bridge ROS-side logic | `ros1_ws/sim_bridge/` (no image rebuild needed, bind mount) |
| Modify Isaac Sim-side logic | `r2s2r/` (same, bind mount — just restart container) |
| Change Docker image dependencies | `docker-full/Dockerfile.*` → `./start.sh --stop` then `docker compose build` |

### 11.3 Docker Resource Usage

| Resource | Name | Purpose | Size |
|----------|------|---------|------|
| Image | `r2s2r-isaac:5.1.0` | Isaac Sim container | ~23 GB |
| Image | `r2s2r-ros1:noetic` | ROS1 + catkin_ws build artifacts | ~5 GB |
| Volume | `docker-full_isaac-data` | Isaac Sim user data | <100 MB |
| Volume | `docker-full_isaac-cache` | Kit cache | ~1 GB |
| Volume | `docker-full_isaac-computecache` | ⭐ Vulkan shader cache (deleting triggers 5–10 min cold start) | ~500 MB |
| Volume | `docker-full_isaac-logs` | Logs | <100 MB |
| Volume | `docker-full_isaac-config` | User config | <100 MB |

### 11.4 Full Cleanup / Reset

```bash
./start.sh --stop                   # Stop containers
docker compose -f docker-full/docker-compose.yml down -v   # Delete volumes (triggers cold start next time)
docker rmi r2s2r-isaac:5.1.0 r2s2r-ros1:noetic            # Delete images (will rebuild/re-pull)
sudo rm -rf ~/.docker/buildx        # Optional: clear buildx cache
```

### 11.5 Common Debug Commands

```bash
# Live-tail Isaac Sim output
docker logs -f r2s2r_isaac
# Enter container interactively
docker exec -it r2s2r_isaac bash
docker exec -it r2s2r_ros1 bash -ic 'bash'   # bash -ic ensures ROS is sourced
# Restart a single service without affecting the other
docker compose -f docker-full/docker-compose.yml restart isaac
# Check ROS topic data flow
docker exec -it r2s2r_ros1 bash -ic "rostopic hz /registered_scan /imu/data /state_estimation"
```

### 11.6 Known Issues / TODO

- **Bash operator precedence**: `docker exec -d bash -c "..."` command chaining in `start.sh` is sensitive to `&` / `&&` / `;` ordering. Test with `which roslaunch` when rewriting.
- **GUI requires X11**: Isaac Sim window won't appear under Wayland / pure SSH. Consider adding headless mode + WebRTC streaming in the future.
- **Go2 initial position** still uses Spot's coordinates (`[55, -2.5, 0.2]`). Manually change `quadruped.position` to `[1.0, 1.0, 0.9]`.
- **Scene fallback**: When USDZ is missing, the WARN is only printed inside the container — consider adding a `grep "WARN: USDZ"` in start.sh to surface the warning on the host terminal.
- **NGC login assumption**: start.sh has a fallback for NGC detection, but first build may fail if nvcr.io tightens access.

---

## Maintainer

Ethan TU — [Manifoldtech](https://www.manifoldtech.cn/)