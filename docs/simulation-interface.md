# Simulator ↔ ONS2 interface contract

This is the contract shared by `ros1_ws/src/sim_bridge` and `ros2_ws/src/sim_bridge_ros2`.
Both runtimes speak the same Isaac TCP protocol (`r2s2r/bridge/protocol.py`, msgpack).

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| pub | `/odin1/odometry` | `nav_msgs/Odometry` | `frame_id=odom`, `child_frame_id=imu` (sensor pose). |
| pub | `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | World-frame cloud, `frame_id=map`. |
| pub | `/odin1/image/compressed` | `sensor_msgs/CompressedImage` | Optional for waypoint navigation. |
| pub | `/odin1/imu` | `sensor_msgs/Imu` | Optional. |
| pub | `/clock` | `rosgraph_msgs/Clock` | Required when navigation nodes use `use_sim_time`. |
| pub | `/state_estimation`, `/registered_scan`, `/camera/image/compressed`, `/imu/data` | legacy | Kept for AL-iLQR and older demos. |
| sub | `/cmd_vel` | `geometry_msgs/Twist` | Sole control input; forwarded to Isaac over TCP. |
| sub | `/sim_reset` | `std_msgs/Empty` | Reset to the configured spawn. |

## TF ownership (`frame_layout: odin`)

| Edge | Real robot publisher | Simulator publisher |
|---|---|---|
| `map→odom` | Odin driver (relocalization) | `sim_bridge` (identity) |
| `odom→imu` | Odin driver | `sim_bridge` (body pose ∘ mount) |
| `imu→odin1_base_link` | ONS2 launch (identity) | ONS2 launch |
| `odin1_base_link→base_link` | ONS2 launch (mount) | ONS2 launch |
| `odin1_base_link→odin1_lidar_link` | ONS2 launch | ONS2 launch |
| `odom→base_link`, `base_link→odin1_base_link` | — | **not published** in `odin` mode |

`frame_layout: legacy` (default) keeps the pre-ONS2 topology:
`map→odom→base_link→{odin1_base_link, lidar1, camera1, imu_link}`.

The `robots/go2_sim/robot.yaml` mount is the exact inverse of the simulator calibration
(sensor at `[0.313, -0.025, 0.086]` in the body frame).

## Clock

The bridge publishes `/clock` from the Isaac simulation clock. Navigation nodes must run
with `use_sim_time:=true`; the ONS2 launch files expose this flag.

## Point cloud coloring

`/odin1/cloud_slam` carries a single `rgb` field (float32, PCL packing, BGR byte order).
With `camera_color: true` the bridge projects the nearest image (pose at image time,
per-pixel nearest-depth z-buffer) onto the points; points outside the frustum fall back
to a source-PCD voxel LUT that is gamma-corrected by `lut_gamma` (0.5 for the heavy scenes).
Images carry the rendered pinhole intrinsics and `body_to_camera`, so
`sensor_suite.camera_render_scale` scales the render and the projection together.
In Foxglove select **BGR (packed)** and set **Color field** to `rgb`; the default
`Distance (auto)` leaves the cloud black.
