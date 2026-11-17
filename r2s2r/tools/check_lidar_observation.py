"""Headless Isaac 5.1 smoke check: python.sh -m r2s2r.tools.check_lidar_observation.

Creates only a synthetic floor and wall. Requires the repository on PYTHONPATH
and an RTX GPU. No ROS server, external assets or network connection is used.
"""
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'width': 320, 'height': 240,
                     'renderer': 'RayTracedLighting'})

try:
    import time
    from pathlib import Path
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.xforms import reset_and_set_xform_ops
    from isaacsim.sensors.rtx import LidarRtx
    from pxr import Gf, UsdGeom
    from r2s2r.core.sensor_suite import create_lidar_writer

    world = World(physics_dt=1/500, rendering_dt=1/50, stage_units_in_meters=1)
    stage = omni.usd.get_context().get_stage()
    for name, position, size in [('Floor', (0,0,-.05), (20,20,.1)),
                                 ('Wall', (3,0,1), (.1,4,2))]:
        cube = UsdGeom.Cube.Define(stage, '/World/'+name)
        cube.GetSizeAttr().Set(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3d(*size))
    profile = Path(__file__).resolve().parents[2]/'lidar_configs/usd/odin1_120x90_0p5.usd'
    add_reference_to_stage(str(profile), '/World/Lidar', prim_type='OmniLidar')
    prim = stage.GetPrimAtPath('/World/Lidar')
    reset_and_set_xform_ops(prim, Gf.Vec3d(.43,0,.40), Gf.Quatd(1,0,0,0))
    for key, value in {'outputFrameOfReference': 'WORLD',
                       'outputMotionCompensationState': 'COMPENSATED'}.items():
        attr = prim.GetAttribute('omni:sensor:Core:'+key)
        assert attr.IsValid() and attr.Set(value) and attr.Get() == value, key
    lidar = LidarRtx(prim_path='/World/Lidar', name='contract_lidar')
    world.reset()
    lidar.initialize()
    writer = create_lidar_writer(lidar.get_render_product_path())
    world.play()
    seen = set()
    for _ in range(600):
        world.step(render=True)
        time.sleep(.02)
        data = writer.latest
        assert 'error' not in data, data.get('error')
        if not data or data['scan_stamp_ns'] in seen:
            continue
        seen.add(data['scan_stamp_ns'])
        age = world.current_time-data['measurement_stamp']
        assert 0 <= age < 1.0, age
        np.testing.assert_allclose(data['lidar_to_world'][:3,3], [.43,0,.40], atol=1e-5)
        points = data['point_cloud']
        wall = points[(np.abs(points[:,1]) < 1) & (points[:,2] > .1) & (points[:,2] < 1.5)]
        assert len(wall) > 100
        wall_x = float(np.median(wall[:,0]))
        assert abs(wall_x-2.95) < .03, wall_x
        print(f'LIDAR_SCAN points={len(points)} age={age:.3f}s wall_x={wall_x:.4f}', flush=True)
        if len(seen) >= 6:
            break
    assert len(seen) >= 6, f'only {len(seen)} unique scans'
    print('LIDAR_CONTRACT PASS: 6 unique scans', flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
