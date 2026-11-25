English | [简体中文](README.zh-CN.md)

# Demo dataset

This directory is where the simulator looks for scene files (`DATASET_DIR`). Docker mounts it read-only at
`/dataset` inside the containers, and scene configurations reference files with container paths such as
`/dataset/<scene>.usdz`.

## Status

Only this README (and the `README.zh-CN.md` / `.gitkeep` placeholders) is tracked in git. **No scan data is
included in the repository.** A small demo environment is being prepared for a zero-barrier first run and will
be distributed separately, outside git history, together with the other large assets.

Large scans, USDZ scenes, and color LUT assets are not tracked in git and are provided separately (release
download or the data channel announced with the project). If no scene file is present, the simulator still
starts, but the environment is empty and there is nothing to navigate.

## Expected layout

Once the demo pack is placed here, the directory should look like this:

```text
dataset/
├── README.md
├── README.zh-CN.md
├── .gitkeep
└── <scene>.usdz        # Isaac scene: 3DGS visuals + collision mesh
```

- `<scene>.usdz` is the only asset type this directory consumes. Use the name that the selected scene
  configuration expects; existing configs reference names such as `office_0923.usdz`,
  `floor2_hall_3dgs.usdz`, `park_inspection_3dgs.usdz`, or the default `tunnel1.usdz`.
- 2D maps, waypoint missions, and color LUTs do **not** live here; see the table below.
- Do not commit files from this directory except the READMEs.

| Asset | Location | Tracked in git |
|---|---|---|
| USDZ scene (visuals + collision) | `dataset/` | no |
| 2D navigation map (PGM/YAML) | `integrations/ons2/maps/` | yes (small maps) |
| Preset waypoint missions | `integrations/ons2/waypoints/` | yes |
| Color LUT (`.npz`) | `integrations/ons2/color_luts/` | no |

## First run

From the repository root:

```bash
cd /path/to/MT-Real2Sim-ons2
export DATASET_DIR=$PWD/dataset
./start.sh --robot go2
```

Other accepted forms of the same pattern:

```bash
# kinematic_car with the default configuration
DATASET_DIR=$PWD/dataset ./start.sh

# Point start.sh at this directory explicitly
./start.sh --scene $PWD/dataset --robot go2
```

With ONS2 navigation (requires the ONS2 repository and the built `isaac_nav_ros2` image):

```bash
DATASET_DIR=$PWD/dataset ./start.sh --robot go2 --runtime ros2 --stack ons2 \
  --map office_0923_sim --ons2-root /path/to/Odin-Nav-Stack2
```

Notes:

- `start.sh` defaults `DATASET_DIR` to `<repo>/dataset` when the variable is not set, and always passes it to
  `docker compose`, which mounts it at `/dataset` (read-only).
- The scene configuration must reference the file by its container path, for example
  `scene.environment.usdz_path: "/dataset/my_scene.usdz"`. Select a configuration with `SIM_CONFIG=...`.
- If the directory is empty or missing, `start.sh` only warns; Isaac Sim then runs without a scanned scene.

## Getting the demo data

See [docs/end-to-end-scan-to-ons2.md](../docs/end-to-end-scan-to-ons2.md) for the complete pipeline that turns
your own Q9000 scan into a USDZ scene, a PGM/YAML map, and ONS2 waypoints. The demo pack simply places an
already prepared USDZ here so that the first run needs no scanning or conversion work.

Use only scan data you are entitled to use, and follow the authorization requirements of the site owner and
data provider.
