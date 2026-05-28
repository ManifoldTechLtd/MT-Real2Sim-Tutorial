# Custom LiDAR Configuration Guide

[简体中文](lidar_guide_cn.md)

This document covers the complete workflow and known pitfalls for custom Isaac Sim RTX LiDAR configuration.

---

## 1. Current LiDAR in Use

**MT-MLXS** — A customized version based on ZVISION ML-Xs.

| Parameter | Value |
|-----------|-------|
| Type | Solid-state (solidState) |
| Horizontal FOV | ≈123° (-61.3° ~ 61.5°) |
| Vertical FOV | ≈32° (-14.0° ~ 17.7°) |
| Points per frame | 27,000 (1/4 of original 108,000) |
| Scan lines | 180 lines × 150 points/line |
| Range | 0.5m ~ 250m |
| Precision | 0.03m resolution, 0.05m accuracy |

**Changes from original ZVISION ML-Xs:**
- `fireTimeNs` all set to zero — disables firing timing, all points return in a single frame (no motion distortion simulation)
- Points per line reduced from 600 to 150 — reduces point cloud transmission latency to RViz

File locations:
- JSON config: `lidar_configs/json/MT_MLXS.json`
- USD file: `lidar_configs/usd/MT_MLXS.usd`
- Intrinsics reference: `r2s2r/configs/calibration/pinhole/intrinsics_lidar1.yaml`

---

## 2. File Structure

```
lidar_configs/
├── json/
│   └── MT_MLXS.json          # LiDAR profile (Isaac Sim RTX JSON format)
└── usd/
    └── MT_MLXS.usd           # Converted USD file (actually loaded at runtime)
```

**Important**: Isaac Sim 5.x only reads USD at runtime, not JSON. JSON is the source file; USD is the build artifact.

---

## 3. Workflow for Creating/Modifying Custom LiDAR

### 3.1 Prepare JSON Configuration

JSON format follows the Isaac Sim RTX LiDAR profile specification. Templates can be obtained from:

- ZVISION official: https://github.com/ZVISION-lidar/ZVISION-lidar.github.io
- Isaac Sim built-in config directory: `${ISAACSIM_PATH}/tools/isaacsim.sensors.rtx/`

Key fields:
```json
{
  "name": "MT-MLXS",
  "class": "sensor",
  "type": "lidar",
  "profile": {
    "scanType": "solidState",
    "nearRangeM": 0.5,
    "farRangeM": 250,
    "numRaysPerLine": [150, 150, ...],       // Points per line
    "emitterStates": [{
      "azimuthDeg": [...],                    // Horizontal angle list
      "elevationDeg": [...],                  // Vertical angle list
      "fireTimeNs": [0, 0, ...]              // Fire time (all zeros = single-frame full return)
    }]
  }
}
```

### 3.2 JSON → USD Conversion

**Must be run in the Isaac Sim Python environment** (inside container or local installation):

```bash
# Inside the Isaac Sim container:
python tools/convert_lidar_json.py lidar_configs/json/MT_MLXS.json

# Output: lidar_configs/usd/MT_MLXS.usd
```

### 3.3 Configuration Reference

Edit `r2s2r/configs/calibration/pinhole/intrinsics_lidar1.yaml`:

```yaml
model: rtx_lidar
config_name: "lidar_configs/json/MT_MLXS.json"
```

When `config_name` ends with `.json`, `sensor_suite.py` automatically searches for the corresponding `.usd` file.

Search logic: given `lidar_configs/json/MT_MLXS.json`, USD is searched in this order:
1. `lidar_configs/json/MT_MLXS.usd` (same directory)
2. `lidar_configs/usd/MT_MLXS.usd` (parallel usd/ directory) ← usually hits

---

## 4. Known Pitfalls

### 4.1 Isaac Sim 5.x No Longer Supports Runtime JSON Reading

Isaac Sim 4.x could point directly to a JSON file via the `sensorModelConfig` attribute.
**Isaac Sim 5.x uses USD prim attributes (`omni:sensor:Core:*`)** — you must generate USD with the conversion tool first.

If you forget to convert, `sensor_suite.py` won't find the USD file and **silently falls back to `Example_Rotary`** (built-in rotary LiDAR), resulting in a completely different point cloud pattern.

### 4.2 Conversion Script Requires Isaac Sim Python Environment

`tools/convert_lidar_json.py` internally calls `isaacsim.SimulationApp` and the official `convert_lidar_json_to_usda.py`. **Cannot be run with standard Python**.

Official tool path:
```
${ISAACSIM_PATH}/tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py
```

If Isaac Sim is not installed at the default path (`~/isaacsim`), set the environment variable:
```bash
export ISAACSIM_PATH=/path/to/isaacsim
```

### 4.3 USD Files Should Be Committed to Version Control

USD is a binary file, but since conversion requires the Isaac Sim environment, it's recommended to **commit USD files to Git** so users don't need to run the conversion during deployment.

### 4.4 JSON Path Resolution

`config_name` in `sensor_suite.py` is a relative path, resolved by searching these directories in order:
1. `r2s2r/configs/` (parent of calibration)
2. `r2s2r/` (project module root)
3. Workspace root (`mt-real2sim-tutorial/`) ← usually hits

If the path can't be found, check Isaac Sim console logs prefixed with `[LiDAR Debug]`.

### 4.5 USD Must Be Regenerated After Modifying JSON

JSON and USD are not automatically synced. After every JSON modification, re-run:
```bash
python tools/convert_lidar_json.py lidar_configs/json/MT_MLXS.json
```

---

## 5. Switching to Isaac Sim Built-in LiDAR

If custom configuration is not needed, you can use a built-in profile name directly:

```yaml
# intrinsics_lidar1.yaml
model: rtx_lidar
config_name: "ZVISION_MLXS"       # Built-in config name, no JSON/USD needed
```

Common built-in profiles:
- `Example_Rotary` — Rotary, 130K+ pts/frame
- `ZVISION_ML30S` — Solid-state, 51K pts, 153°×73°, 45m
- `ZVISION_MLXS` — Solid-state, 108K pts, 123°×32°, 250m
