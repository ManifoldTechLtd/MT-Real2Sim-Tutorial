English | [简体中文](end-to-end-scan-to-ons2.zh-CN.md)

# From Q9000 Scanning to Isaac Sim Navigation

This tutorial builds an Isaac Sim scene from a site you scanned yourself and runs ONS2 navigation with Go2 in that scene.
Every step states its **input** and **output**, so you can execute them in order.

Applies to: Isaac Sim 5.1, ROS 2 Jazzy, ONS2, Go2 + ODIN1.

Before starting: scene and map files are large, so keep them in your own local directory.
This tutorial uses two repositories: `MT-Real2Sim-Tutorial` (simulation) and `Odin-Nav-Stack2` (navigation).
The ONS2 version you use must include the Go2 simulation configuration; if it does not, obtain a version with simulation support first.

## 0. Overview

```text
Q9000 scan
  → export scan project
  → MindCloudX AI training
      ├─ cloud.pcd            → navigation map PGM/YAML
      ├─ 3dgs.ply + mesh.ply  → scene.usdz
      └─ map.bin              → only for relocalization on the real robot
  → configure scene and spawn point
  → launch Isaac Sim + ONS2
  → pick waypoints, robot navigates
```

| File | Purpose |
|---|---|
| `cloud.pcd` | Generate the 2D navigation map; optionally generate a color LUT |
| `3dgs.ply` | Generate the scene visuals (Gaussian splatting) |
| `mesh.ply` | Generate collision geometry (floor, walls) |
| `scene.usdz` | Final file placed into the Isaac scene |
| `my_scene_sim.pgm/.yaml` | 2D map used by navigation |
| `my_scene_meta.json` | Floor height, suggested spawn point |
| `my_scene_sim.npz` | Optional point-cloud fallback color LUT |
| `map.bin` | Odin relocalization map for the real robot; not needed for simulation |

Below, `my_scene` denotes the scene name and `my_scene_sim` the navigation map name.

## 1. Prepare directories and terminals

**Input:** a development computer with Docker installed, and both repositories.

Open a terminal, adjust the commands to your actual paths, and run:

```bash
export SIM_ROOT=/path/to/MT-Real2Sim-Tutorial
export ONS2_ROOT=/path/to/Odin-Nav-Stack2
export SCENE_ROOT="$HOME/data/my_scene"
export DATASET_DIR="$SIM_ROOT/dataset"

export ROS_DOMAIN_ID=77

mkdir -p "$SCENE_ROOT/ply" "$SCENE_ROOT/mesh" "$SCENE_ROOT/out" "$DATASET_DIR"
cd "$SIM_ROOT"
```

The following commands are assumed to run in this terminal. `DATASET_DIR` is the directory from which Isaac reads scene files.

**Output:** both repository paths and the scene working directory are ready.

## 2. Scan the site with Q9000

**Input:** Q9000, phone, MindCloud Go.

1. Install **MindCloud Go** on the phone.
2. Check that the Q9000 cables and power supply are working properly.
3. Power on the Q9000 and connect the phone to the Q9000 Wi-Fi. The network name usually contains the last four digits of the device serial number; use the password given in the device instructions.
4. Open the app, tap **Connect device**, and wait for the connection to succeed.
5. Tap **Start scan**, then the green play button, and enter the project name `my_scene`.
6. Hold the Q9000 and walk slowly through the area to be navigated. Walk through corridors, doorways, and corners several times, scan feature-rich areas, and finally return to an already scanned area (for scanning technique you can ask a Manifold Tech after-sales FAE).
7. Tap stop and confirm that the project has been saved.
8. Connect the Q9000 data port to the computer with a USB 3.0 cable, or use the device export function, and copy the entire scan project to the computer.

Do not walk too fast while scanning; avoid motion blur and missed areas. Glass, mirrors, and reflective floors easily produce incorrect points, so check the map afterwards.

**Output:** the complete scan project folder on the computer.

## 3. Train in MindCloudX AI and download

**Input:** the complete scan project from step 2.

1. Open https://mindcloudx.ai/ and log in or register.
2. Click **Upload dataset**, select the scan project folder, and upload it.
   Copying the project to the computer's local disk before uploading is recommended, to avoid disconnection of the USB drive or device.
3. After the upload completes, open the project with the same name. We need the **point cloud, 3DGS, and Mesh**:
   create a training task on each corresponding tab.
4. Keep the training parameters at their defaults, enable "dynamic object removal" if needed, wait for completion, and preview the results.
5. Download the files:

```text
my_scene/
├── cloud.pcd
├── ply/
│   └── 3dgs.ply      # download the Z-up version
└── mesh/
    └── mesh.ply
```

6. If you also want to run navigation on a **real robot**, additionally download `map.bin` from the same project; it is not needed for simulation only.

Confirm that the three files come from the same project; if the file names differ, rename them and place them in the following locations:

```bash
cp /path/to/cloud.pcd "$SCENE_ROOT/cloud.pcd"
cp /path/to/3dgs.ply  "$SCENE_ROOT/ply/3dgs.ply"
cp /path/to/mesh.ply  "$SCENE_ROOT/mesh/mesh.ply"
```

**Output:** `cloud.pcd`, `ply/3dgs.ply`, `mesh/mesh.ply`, and optionally `map.bin`.

## 4. Convert 3DGS and Mesh to USDZ

**Input:** `ply/3dgs.ply`, `mesh/mesh.ply`.
If you already have a usable USDZ, you can skip this section and go directly to step 5 to generate the map.

The conversion scripts are in `docs/mt-r2s/mindx2isaacsim/`. Install the dependencies first:

```bash
cd "$SIM_ROOT"
python3 -m venv .venv-scene
source .venv-scene/bin/activate
pip install -r docs/mt-r2s/mindx2isaacsim/requirements.txt
pip install pyyaml python-lzf
```

Python 3.10–3.12 is required. If the system reports that `venv` is missing, install the matching `python3-venv` package first.

Run the conversion:

```bash
python docs/mt-r2s/mindx2isaacsim/export_usdz.py \
  --ply_dir "$SCENE_ROOT/ply" \
  --mesh_dir "$SCENE_ROOT/mesh" \
  -o "$SCENE_ROOT/usdz" \
  --simplify_faces 500000 \
  --max_dist 200

cp "$SCENE_ROOT/usdz/scene.usdz" "$DATASET_DIR/my_scene.usdz"
```

Parameter reference:

| Parameter | Meaning | Recommendation |
|---|---|---|
| `--ply_dir` | Directory containing the 3DGS PLY | Put `3dgs.ply` there |
| `--mesh_dir` | Directory containing the Mesh PLY | Put `mesh.ply` there |
| `-o` | USDZ output directory | Produces `scene.usdz` |
| `--simplify_faces` | Target face count for the decimated collision mesh | Default 500k; too few may miss collisions, too many may fail to import |
| `--max_dist` | Discard Gaussians farther than this distance from the origin | Default 200 m; `0` means no filtering |
| `--mesh_visible` | Whether the collision mesh is visible | Invisible by default; used only for physics and LiDAR |

The collision mesh uses `collision_approx=none` by default; do not change it.
Do not add `--y_up` for Z-up data; that option is only for legacy Y-up data.

**Output:** `$DATASET_DIR/my_scene.usdz`.
This file provides both visuals and collision; a successful export does not mean correct collision, so after startup you still need to check that the robot does not fall through the floor or walk through walls.

## 5. Generate the navigation map (key step)

**Input:** `cloud.pcd` and `mesh.ply` from the same site.

ONS2 navigation needs a 2D PGM/YAML map and cannot use the USDZ directly. Use the map tool bundled with the project here.
If step 4 already produced the USDZ, you must add `--skip-usdz`; otherwise the tool generates another USDZ that contains only the mesh.
If you skipped step 4, first install the dependencies from step 4 and run `source "$SIM_ROOT/.venv-scene/bin/activate"`.

```bash
cd "$SIM_ROOT"
source .venv-scene/bin/activate

python tools/build_scan_scene.py \
  --name my_scene \
  --mesh "$SCENE_ROOT/mesh/mesh.ply" \
  --pcd "$SCENE_ROOT/cloud.pcd" \
  --out-dir "$SCENE_ROOT/out" \
  --skip-usdz \
  --resolution 0.05

cp "$SCENE_ROOT/out/my_scene_sim.pgm" \
   "$SCENE_ROOT/out/my_scene_sim.yaml" integrations/ons2/maps/
```

### 5.1 Parameter reference

| Parameter | Meaning |
|---|---|
| `--name` | Scene name. `--name my_scene` outputs `my_scene_sim.pgm/.yaml` and `my_scene_meta.json` |
| `--mesh` | Mesh file. Must be provided on the command line; with `--skip-usdz` the USDZ is not rebuilt, but the argument is still required to satisfy the command format |
| `--pcd` | Scan point cloud, used to generate the map |
| `--out-dir` | Output directory |
| `--skip-usdz` | Generate the map only; use when a USDZ already exists |
| `--resolution` | Meters per cell. `0.05` means 5 cm; use `0.1` for very large maps |
| `--fixed-floor` | Z height in the original point cloud of the floor to navigate; when omitted, the lowest ~2% of points are taken as the ground |
| `--min-height` / `--max-height` | Obstacle height range relative to the floor, defaults `0.30 / 0.90` m |
| `--free-min` / `--free-max` | Traversable height range relative to the floor, defaults `-0.30 / 3.00` m |
| `--color-voxel`, `--color-stride`, `--color-max-points` | Used only when the mesh USDZ is generated automatically; not needed with `--skip-usdz` |

**How to determine `--fixed-floor`:**
For a single-floor site, first generate the map with the defaults and open the PGM to check; if the ground is detected correctly, this parameter is not needed.
For multi-floor or atrium sites, first look at `floor_z` in `my_scene_meta.json`, or use a point-cloud viewer to find the height of the floor the robot will drive on, then pass that Z value to `--fixed-floor`.
The other options `--min-height/--max-height` and `--free-min/--free-max` are all relative to this floor;
adjust them according to on-site obstacles and the slab thickness. Do not project other floors onto the current floor.

### 5.2 Tool output

Using `--name my_scene` as an example:

- `my_scene_sim.pgm`: white = traversable, black = obstacle, gray = unknown;
- `my_scene_sim.yaml`: map resolution and origin;
- `my_scene_meta.json`: `floor_z`, `scene_position_z`, `spawn_xy`, etc.

The map and the USDZ share the same X/Y coordinate frame. If the scene is shifted along X/Y, the map, spawn point, and waypoints must be shifted together;
do not move only the USDZ.

### 5.3 Default ground, floor height, and spawn point

This is an error-prone point; first distinguish three heights:

1. **Default ground**: a flat fallback ground in Isaac, positioned at world `z=0` plus
   `scene.ground_z_offset` (scanned scenes commonly use `-0.02`). It is not the floor height captured by the scan.
2. **Scene floor**: the floor in the scanned point cloud. The map tool reports `floor_z` and a suggested
   `scene_position_z`, where `scene_position_z = -floor_z`.
3. **Robot spawn height**: the robot's initial position. It is not the floor height but the **drop height** above the floor.

Correct approach:

```text
Set scene.environment.position.z to scene_position_z
    → the scanned floor lands at z=0
Set the X/Y of quadruped.position to spawn_xy
    → the robot stands in a blank area of the map
Set quadruped.position.z to about 0.5
    → the robot drops a little above the floor so locomotion can settle
```

For example: if `floor_z` in the meta is `2.00`, set `scene_position_z=-2.00`,
put `-2.00` in the scene configuration's `position.z`, and keep the spawn `position.z` at `0.5`.
If `floor_z` is `-0.50`, enter `+0.50`. The key is to make the scanned floor land at `z=0`,
not to cancel out the floor height with the spawn height.

Configuration example:

```yaml
scene:
  environment:
    position: [0.0, 0.0, 0.0]       # set to scene_position_z from the meta
  use_default_ground: true         # fallback when the scanned mesh has gaps
  ground_z_offset: -0.02
  physics:
    collision_approximation: none

quadruped:
  position: [0.0, 0.0, 0.5]        # set X/Y to spawn_xy; Z is the drop height
```

- **Do not spawn very high**: 0.4–0.6 m is usually enough for Go2 to drop and stand;
  dropping from above 1 m may bounce, tip over, or damage the robot.
- **Do not spawn inside the floor**: if Z is lower than the floor, the robot may be ejected or clip through.
- **Do not guess the floor by dragging the USDZ**: dragging in the GUI is only for rough viewing; for precise placement change the
  numbers in `scene.environment.position`. Dragging does not move the map and waypoints automatically.
- **What if there is no `my_scene_meta.json`**: use a point-cloud viewer or the map tool to find the floor Z;
  in Isaac you can also place a temporary cube/marker on the floor and read its Z value.
  Do not infer the floor height from the robot's spawn height.
- **How to verify**: after launch, wait for the robot to stabilize and read the z of `/odin1/odometry` or
  `/state_estimation`. The robot should stabilize about 0.3–0.5 m above the floor;
  if z keeps decreasing, the collision ground is missing or the spawn is over a hole;
  if it falls through or is ejected immediately, check `scene_position_z` and the spawn Z.
- **When to use the default ground**: when the scanned mesh lacks a complete floor and has local holes, `use_default_ground: true`
  places a flat fallback ground at `ground_z_offset`. For multi-floor, ramp, or large-area scenes, disable it
  and use the USDZ collision mesh instead, so the fallback ground does not hide real lower or higher areas.

**Output:** `integrations/ons2/maps/my_scene_sim.pgm` and `.yaml`,
plus the floor/spawn information in `my_scene_meta.json`.

## 6. Configure the Isaac scene

**Input:** `my_scene.usdz`, `my_scene_sim.pgm/.yaml`, `my_scene_meta.json`.

Copy the generic Go2 configuration as a template:

```bash
cd "$SIM_ROOT"
cp r2s2r/configs/go2.yaml r2s2r/configs/go2_my_scene.yaml
```

Open `go2_my_scene.yaml` and change the following fields; leave the other fields at their template values for now:

```yaml
simulation:
  headless: false
  window_width: 1280
  window_height: 720

scene:
  environment:
    enabled: true
    usdz_path: "/dataset/my_scene.usdz"   # path inside the container, not an absolute path on your computer
    position: [0.0, 0.0, 0.0]             # set to scene_position_z from the meta
    add_collision: true
    keep_invisible_collision_meshes: true
  use_default_ground: true
  ground_z_offset: -0.02
  physics:
    collision_approximation: none

quadruped:
  robot_type: go2
  prim_path: /World/Quadruped
  position: [0.0, 0.0, 0.5]               # set X/Y to spawn_xy; Z is the drop height

sensor_suite:
  enabled: true
  calibration_dir: calibration/odin1_go2
  camera_render_scale: 0.5

bridge:
  pointcloud_rate: 10
  image_rate: 5
  image_quality: 70
```

Notes:

- `/dataset/my_scene.usdz` is the path inside the container, corresponding to `DATASET_DIR/my_scene.usdz` on your computer;
- `scene_position_z` comes from `my_scene_meta.json`; it is not the spawn Z;
- use `spawn_xy` for the spawn X/Y, and make sure it is inside the white area of the map and at least about 1 m away from obstacles;
- use about 0.5 for the spawn Z so the robot drops and stands;
- if the scene has no complete floor, keep `use_default_ground: true` and `ground_z_offset: -0.02`;
- robot standing height, camera rate, and resolution can be adjusted for machine performance and visual quality needs.

**Output:** `r2s2r/configs/go2_my_scene.yaml`.

Note: later we will add the ASTRALL sw01 model and the Unitree As2 with its locomotion controller for simulation training. The repository also provides a simple box-shaped differential-drive car model for validating differential-drive algorithms.

## 7. Generate the point cloud color LUT (optional)

**Input:** a colored `cloud.pcd`, and the X/Y/Z translation of `scene.environment.position` from step 6.

Optional; enabling it gives a better color appearance.

```bash
cd "$SIM_ROOT"
source .venv-scene/bin/activate

python tools/build_color_lut.py \
  --pcd "$SCENE_ROOT/cloud.pcd" \
  --out integrations/ons2/color_luts/my_scene_sim.npz \
  --voxel 0.1 --stride 2 --max-points 25000000 \
  --translation 0 0 0.0
```

`--translation` must exactly match the X/Y/Z of `scene.environment.position` in step 6.
The `0 0 0.0` in the example is only a placeholder; use `scene_position_z` from the meta for Z, and fill in the matching X/Y if you translated X/Y as well.
The file name must match the map name `my_scene_sim`.
Running without this file also works; points not covered by the camera then use the default color.

**Output:** `integrations/ons2/color_luts/my_scene_sim.npz`.

## 8. Build the runtime environment and compile navigation

**Input:** both repositories, Docker, an NVIDIA GPU, and the scene configuration.

Build the images:

```bash
cd "$SIM_ROOT"
docker build -f docker/Dockerfile.isaac -t r2s2r-isaac:5.1.0 .
docker compose -f docker/docker-compose.yml build ros2
```

If pulling the Isaac base image requires a login, sign in to the terminal as described by NVIDIA NGC; do not write accounts or tokens into the repository.

Start the ROS 2 container and compile the navigation workspace:

```bash
cd "$SIM_ROOT"
ONS2_ROOT="$ONS2_ROOT" DATASET_DIR="$DATASET_DIR" \
  docker compose -f docker/docker-compose.yml up -d ros2

docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /ons2/ros2_ws &&
  colcon build --symlink-install --packages-select \
    nav_interfaces atomic_move map_planner sliding_map neupan_ros mission_manager \
    ons2_bringup sim_bridge_ros2'
```

**Output:** both images and the compiled navigation workspace; if compilation reports errors, fix them first and do not continue to launch.

## 9. Launch the simulation, select waypoints, and verify

**Input:** the scene configuration from step 6, the map from step 5, and the build result from step 8.

```bash
cd "$SIM_ROOT"
SIM_CONFIG=r2s2r/configs/go2_my_scene.yaml \
DATASET_DIR="$DATASET_DIR" \
./start.sh --robot go2 --runtime ros2 --stack ons2 \
  --map my_scene_sim \
  --ons2-root "$ONS2_ROOT"
```

Wait for the Isaac window to load the scene. If the window is paused, click **PLAY**.
After launch, the navigation console lives in tmux inside the container:

```bash
docker exec -it isaac_nav_ros2 tmux attach -t ons2_sim
```

### Selecting waypoints

Connect Foxglove to port 8765 on the local machine and open the 3D panel:

1. Visualize the map `/map`, the point cloud `/odin1/cloud_slam`, the image `/odin1/image/compressed`, and any other topics you need;
2. Set the publish topic to **`/waypoint_pick`** and the frame to **`map`** (consistent with ONS2);
3. Use the Pose tool to pick points in the white area of the map;
4. Create a single-point or multi-point mission in the console and start it.

You can also use preset waypoints. Create `integrations/ons2/waypoints/my_scene_sim.yaml`:

```yaml
waypoints:
  - {name: P1, x: 1.0, y: 0.0, yaw: 0.0, dwell_sec: 2.0}
  - {name: P2, x: 2.0, y: 1.0, yaw: 1.57, dwell_sec: 2.0}
```

The coordinates are a format example only and must be replaced with safe positions in your own map; `yaw` is in radians and `dwell_sec` is the dwell time in seconds.
Send the mission:

```bash
bash integrations/ons2/waypoints/send_waypoints.sh \
  --runtime ros2 --scene my_scene_sim
```

### Point cloud color

Expand the `/odin1/cloud_slam` settings:

- choose **BGR (packed)** as the color mode;
- choose **`rgb`** as the color field; do not leave it on "distance (automatic)".

### Verification

```bash
docker exec -it isaac_nav_ros2 bash
source /opt/ros/jazzy/setup.bash
source /ons2/ros2_ws/install/setup.bash
ros2 topic echo /waypoint_manager/status --once
```

The mission should progress from `NAVIGATING` through `DWELLING` to `COMPLETED`.
Images, point cloud, and map should keep updating; when the simulation slows down, actual rates may be below the configured values.

Stop:

```bash
cd "$SIM_ROOT"
./start.sh --stop
```

**Output:** the robot moves along the waypoints and the mission completes.

## 10. Differences between real-robot navigation and simulation

This tutorial focuses on simulation. If you also want to navigate on a real robot with the same scan data, additional preparation is needed:

| | Simulation | Real robot |
|---|---|---|
| Scene | USDZ | Real site |
| Localization | Simulated pose | Odin localization |
| 2D map | PGM/YAML | PGM/YAML |
| Relocalization map `.bin` | Not needed | Configured per the Odin driver requirements |
| Control | Simulated `/cmd_vel` | Real chassis |

When converting PCD to PGM/YAML for the real robot, use `pcd_to_pgm.py` from the navigation repository,
and set the LiDAR height and slicing range according to the real mounting height. PCD, PGM, YAML, and BIN must share the same base name,
and the `image` field in the YAML must match as well. Do not reuse the simulation spawn point as the real-robot spawn/localization position.

## 11. FAQ

| Symptom | Check |
|---|---|
| Cannot find the `go2_sim` simulation configuration | Whether the ONS2 version in use includes Go2 simulation support |
| `docker exec` cannot find the container | Start the ROS 2 container in step 8 first |
| USDZ not found | Whether `DATASET_DIR` is correct; whether the configuration uses the in-container `/dataset/` path |
| Map and point cloud misaligned | Whether they come from the same project; whether the scene, map, and spawn X/Y were handled together |
| Start point unusable, `START_OCCUPIED` | Whether the spawn point is in the white area and far enough from obstacles |
| Robot falls through the floor or is ejected after spawning | Check `scene_position_z`, the spawn Z, and the collision ground; do not only adjust the spawn height |
| Robot penetrates the floor | Whether the collision mesh generated from 3DGS has holes, and whether the face count is appropriate |
| Simulation is very slow | Lower the camera scale, image rate, or GUI resolution; do not mask stutter by raising speed |
| TF expired or pose jumps | Navigation nodes use the simulation clock; avoid sharing a domain with other ROS 2 systems |


For integration with ONS2, see the [ONS2 integration directory](../integrations/ons2/README.md).

## 12. References

- Official tutorial and scripts:
  https://github.com/ManifoldTechLtd/MT-Real2Sim-Tutorial/tree/master/docs/mt-r2s
- ONS2 navigation documentation: `Odin-Nav-Stack2/docs/simulation/04_simulation.md` / `.zh-CN.md`

> Use only scan data you are entitled to use, and comply with the authorization requirements of the site owner and data provider.
