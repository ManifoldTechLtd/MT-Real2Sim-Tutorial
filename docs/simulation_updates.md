# Go2, lidar and bridge updates

This document describes the simulation changes
only: Go2 MoE-CTS + PI, ODIN1 lidar/calibration, synchronized point-cloud
observations, simulation clock/ROS bridge, TCP/teleop and performance diagnostics.
The original navigation packages remain unchanged. Local topology/exploration/
trajectory-planning code, external planner mounts/dependencies, movement-model
changes and dynamic obstacles are excluded.

## Start and calibration

Use ./start.sh --robot go2. The Go2 PI is enabled by default.
Main configuration selects calibration/odin1_go2 for Go2 and calibration/odin1
otherwise; the bridge selects the matching odin1_go2/odin1 profile.
The ODIN1 camera is disabled. Additional raster/scatter profiles and
tools/gen_raster_lidar_json.py / tools/convert_lidar_in_docker.sh are included
for generating and converting alternate scans. The lidar is 120°×90° with an approximately
0.5° angle grid; the bundled JSON and USD are used without online conversion.
The Go2 lidar origin is [0.285, 0, 0.12] in base; the other profile uses
[0.43, 0, 0.10]. Sensor-internal relative calibration is retained.

The kinematic vehicle shape and movement settings remain those of the simulator baseline.
Body-return filtering supports both boxes and double cylinders, but changing
vehicle geometry is outside this scope.

## Observation and clock contract

Restart Isaac and sim_bridge together: CLOCK (0x06) carries simulation time
and bridge.launch enables /use_sim_time. Pose, sensor publication periods and
timestamps use simulation time. Velocity comes from pose differences rather
than echoing command velocity. Command receive timeouts still use wall time.

The RTX writer captures XYZ, scan transform and simulation timestamp together.
Clouds include the measured body_pose so the bridge can insert historical TF.
Repeated, stale, invalid and pre-reset observations are rejected.
The selected simultaneous-fire profile must span at most 1 ms; this is not a
general per-ray deskew implementation for spinning lidars.

TCP drains messages until blocked while preserving partial frames.
Keyboard teleop yields while another node publishes cmd_vel.

## Performance and diagnostics

simulation.render_hz defaults to 25; Go2 policy inference remains 50 Hz.
Compose exposes PHYSX_GPU_DYNAMICS, PHYSX_DEVICE, GPU_MAX_PARTITIONS,
COLLISION_APPROX and CONTACT_OFFSET as optional investigation overrides.

tools/capture_hang.sh [container] records system/thread state and stack traces
under hang_logs/ or HANG_LOG_DIR. It may install gdb/elfutils/py-spy in the target
container and briefly pause the process; run it only for an observed hang.
The Isaac container has SYS_PTRACE for this opt-in diagnostic tool.
Sensor debug output goes to the process temporary directory, allowing the
source tree to remain mounted read-only. No captured logs, memory maps,
recordings or private scene paths are committed.

Heavy NuRec scenes expose three quality knobs while keeping the GUI enabled:
`sensor_suite.camera_render_scale` (0.5 renders the camera at 800x648 and scales
the projection intrinsics), `bridge.image_rate` / `image_quality` (5 Hz / JPEG 70
for the Go2 park and floor2 profiles), and `simulation.window_width/height`
(1280x720 for the GUI viewport). Measured on the park scene: about 3.7 Hz cloud,
0.46x wall-clock simulation speed, and the P1-P3 mission completed in ~180 s.
Raise these values when visual quality matters more than throughput.

Cameras default to `clippingRange=(0.05, 1000)`; the USD default near plane of
1 m clipped the near ground/robot and produced a dark band with triangular edges
at the bottom of low-mounted camera images. Override per scene with
`sensor_suite.camera_clip_near` / `camera_clip_far`; the before/after values are
printed at camera creation.

## Checks

With Python, NumPy, Torch, PyYAML and msgpack available, from the project root:

    python3 -m unittest r2s2r.test_go2_moe_runtime r2s2r.test_observation r2s2r.test_sim_clock r2s2r.test_sim_pose_feedback r2s2r.test_sensor_calibration -v
    python3 ros1_ws/src/nav_demo/scripts/test_teleop_keyboard.py

Inside Isaac Sim 5.1 with the repository on PYTHONPATH and an RTX GPU:

    /isaac-sim/python.sh -m r2s2r.tools.check_lidar_observation
    /isaac-sim/python.sh r2s2r/tools/test_go2_moe_isaac.py --duration 20

See r2s2r/core/assets/go2/README.md for the policy contract and staircase limit.

## Validation (2026-11-17)

- 25 runtime/observation/clock/pose/calibration checks and 3 teleop checks passed.
- Isaac Sim 5.1: stand, forward, positive and negative yaw each completed an
  8-second command segment plus warmup without a fall.
- RTX lidar: 6 distinct synchronized scans passed; each had 30,819 valid
  returns, measurement age 0.020 s, and synthetic wall X=2.9500 m.
- JSON, YAML, ROS XML, shell syntax, Compose configuration, policy/USD hashes
  and the generated 3×3 lidar profile passed validation.
- Upstream navigation packages, vehicle geometry and ROS Docker dependencies
  remain unchanged. Navigation/exploration code and recordings are not included.
