# Point-cloud coloring (LUT fallback, not tracked in git)

`/odin1/cloud_slam` is colored in two stages:

1. **Camera projection (default for the ROS2 profile).** Each scan is projected
   through the rendered pinhole camera (`intrinsics` + `body_to_camera` ride on
   the image message) using the pose at the image timestamp. Points inside the
   camera frustum get live RGB from the JPEG frame; nearest depth wins per pixel.
   This mirrors how the real Odin colors its SLAM cloud from the RGB stream.
2. **Voxel color LUT (fallback).** Points outside the camera view (behind the
   robot, beyond the frustum) fall back to a color table built from the source
   PCD, or neutral gray when no LUT is loaded.

> The LUT is offline source-PCD coloring, not camera fusion. PCD `TYPE F`
> stores packed RGB bits in float32 fields: reinterpret as uint32, never
> numerically cast the float values. Both floor2 and park exports contain
> usable colors. Apply the same scene translation when building each table;
> unmatched returns are neutral gray. `build_color_lut.py` rejects genuinely
> all-zero tables.

```bash
# second floor hall
python3 tools/build_color_lut.py \
  --pcd /path/to/inspectionMap/5min2楼大厅/3dgs-pointcloud-1123-1737.pcd \
  --out integrations/ons2/color_luts/floor2_hall_ground_sim.npz \
  --voxel 0.1 --stride 2 --max-points 25000000 --translation 0 0 1.35

# park inspection
python3 tools/build_color_lut.py \
  --pcd /path/to/inspectionMap/园区巡检导航地图/q9000-点云.pcd \
  --out integrations/ons2/color_luts/park_inspection_sim.npz \
  --voxel 0.1 --stride 17 --max-points 16000000 --translation 0 88.919562 0.531
```

`start.sh --stack ons2` passes
`color_lut_path:=/integrations/ons2/color_luts/<MAP_NAME>.npz` to the bridge.
If the file is missing the bridge publishes uncolored clouds (or camera-colored
clouds when `camera_color: true` and an image is available). Camera coloring is
enabled by `camera_color: true` in `ros2_ws/src/sim_bridge_ros2/config/go2_sim.yaml`;
the FishPoly model is not projected yet, so only pinhole cameras are colored.

Colored clouds carry a single `rgb` field (float32, PCL packing, BGR byte order,
identical to the real Odin cloud). In the Foxglove 3D panel expand
`/odin1/cloud_slam`, select **BGR (packed)**, and set **Color field** (颜色字段)
from `Distance (auto)` to **`rgb`** — otherwise the points render black even
though the message contains color. Select the live playback time, not a paused
frame. The fallback LUT is gamma-corrected (`lut_gamma: 0.5`, configurable)
because the source-PCD colors are about twice as dark as the camera samples on
the same points. To isolate viewer problems from the bridge, publish the
synthetic test topics/rows and enable them in the 3D panel:

```bash
python3 /integrations/ons2/tools/publish_color_test.py --ros-args -p use_sim_time:=true
```

`/color_test_rgb` (rgb UINT32), `/color_test_rgb_f32` (rgb FLOAT32),
`/color_test_rgba` (rgba UINT32) and `/color_test_markers` (plain RGBA spheres)
separate packed-field decoding from playback/transform problems.

