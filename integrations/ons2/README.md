# ONS2 simulation integration (sim-side assets)

This directory holds the simulator-side pieces for running the Odin Navigation Stack 2
(ONS2) against Isaac Sim. **Usage documentation lives in the ONS2 repository:**
`../Odin-Nav-Stack2/docs/simulation/04_simulation.md` (and `.zh-CN.md`).
The full topic / TF contract is in [`../../docs/simulation-interface.md`](../../docs/simulation-interface.md).

## Layout

| Path | Purpose |
|---|---|
| `maps/office_0923_sim.{pgm,yaml}` | 2D map aligned to `dataset/office_0923.usdz` (demo). |
| `maps/office_0923_v3partial.pgm` | Original partial map the demo map was derived from (provenance). |
| `waypoints/office_0923_sim.yaml` | 3-waypoint mission starting from the spawn area. |
| `waypoints/office_0923_return5.yaml` | 5-waypoint return route from the end of the 3-waypoint mission. |
| `waypoints/send_waypoints.sh` | Sends a preset mission through `/waypoint_manager/start_mission`. |
| `smoke/mock_tcp_bridge.py` | Minimal Isaac TCP stand-in for wiring checks without a GPU. |
| `tools/` | Reserved for `gen_scene_map.py` (USDZ → PGM pipeline). |

## Map provenance

`office_0923_v3partial.pgm` is a partial scanned-map derivative used to build the demo map: it is mostly
`unknown` (205). ONS2's planner treats non-free cells as blocked and relocates goals
that lack clearance, so the demo map fills `unknown` with `free` while keeping walls
(0) and doors (128) blocked. Goals closer than ~0.55 m to a blocked cell are relocated
by the planner; pick waypoints with ≥1.1 m clearance (validated with `/map_planner/plan`).

Scenes (`.usdz`), site maps and Odin `.bin` relocation maps are **not** part of this
repository. Point `DATASET_DIR` at a directory that contains `office_0923.usdz`.

## Verified flow (ROS 2, November 2026)

```bash
# 1) build the runtime image once
cd docker && docker compose build ros2 && cd ..

# 2) compile the ONS2 workspace inside the container (see ONS2 docs for the exact list)
ONS2_ROOT=/path/to/Odin-Nav-Stack2 docker compose -f docker/docker-compose.yml up -d ros2
docker exec isaac_nav_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ons2/ros2_ws &&
  colcon build --symlink-install --packages-select \
    nav_interfaces atomic_move map_planner sliding_map neupan_ros mission_manager \
    ons2_bringup sim_bridge_ros2'

# 3) start Isaac (office scene, headless is fine) + bridge + ONS2
DATASET_DIR=/path/to/dataset ./start.sh --robot go2 --runtime ros2 --stack ons2 --map office_0923_sim

# 4) send the preset waypoint mission
bash integrations/ons2/waypoints/send_waypoints.sh --runtime ros2 --scene office_0923_sim
```

Verified missions:

* `mission_3939409911947` — 3 waypoints, `COMPLETED`.
* `mission_4027849909970` — 5 waypoints, `COMPLETED` (final goal relocated 0.6 m by the
  planner due to map clearance; expected behavior).
