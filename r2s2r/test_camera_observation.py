import numpy as np
import pytest

from r2s2r.core import observation


def test_camera_batch_preserves_capture_time_and_owns_rgb_data():
    rgba = np.array([[[10, 20, 30, 255], [40, 50, 60, 255]]], dtype=np.uint8)
    frame = {'rgb': rgba, 'IsaacReadSimulationTime': {'simulationTime': 2.25}}
    result = observation.decode_camera_frame(frame)
    rgba[:] = 0
    assert result['measurement_stamp'] == 2.25
    np.testing.assert_array_equal(result['rgb'], [[[10, 20, 30], [40, 50, 60]]])
    assert result['rgb'].flags.c_contiguous
    assert result['rgb'].flags.owndata


@pytest.mark.parametrize('image', [np.zeros((2, 2), dtype=np.uint8),
                                    np.zeros((2, 2, 3), dtype=np.float32)])
def test_camera_batch_rejects_invalid_rgb_layout(image):
    with pytest.raises(ValueError):
        observation.decode_camera_frame({'rgb': image, 'IsaacReadSimulationTime': {'simulationTime': 2.25}})


def test_camera_capture_gate_rejects_duplicates_stale_and_future_frames():
    assert observation.fresh_camera_frame({'measurement_stamp': 2.25}, 2.0, 2.3, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 2.25}, 2.25, 2.3, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 2.25}, 2.0, 4.0, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 2.25}, 2.0, 2.0, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 2.25}, 2.0, 2.3, 2.5)


def test_camera_batch_copies_noncontiguous_three_channel_rgb():
    rgb = np.arange(36, dtype=np.uint8).reshape(2, 6, 3)[:, ::2, :]
    result = observation.decode_camera_frame(
        {'rgb': rgb, 'IsaacReadSimulationTime': {'simulationTime': 0.5}})
    np.testing.assert_array_equal(result['rgb'], rgb)
    assert result['rgb'].flags.c_contiguous and result['rgb'].flags.owndata
    assert not np.shares_memory(result['rgb'], rgb)


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), -0.1])
def test_camera_batch_rejects_invalid_capture_time(stamp):
    with pytest.raises(ValueError):
        observation.decode_camera_frame({'rgb': np.zeros((1, 1, 3), dtype=np.uint8),
                                         'IsaacReadSimulationTime': {'simulationTime': stamp}})


@pytest.mark.parametrize('image', [np.zeros((1, 1, 2), dtype=np.uint8),
                                    np.zeros((1, 1, 5), dtype=np.uint8),
                                    np.zeros((0, 1, 3), dtype=np.uint8)])
def test_camera_batch_rejects_empty_or_wrong_channel_count(image):
    with pytest.raises(ValueError):
        observation.decode_camera_frame({'rgb': image,
                                         'IsaacReadSimulationTime': {'simulationTime': 2.25}})


def test_camera_capture_gate_handles_missing_invalid_and_boundary_stamps():
    assert not observation.fresh_camera_frame({}, -1, 2.0, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': float('nan')}, -1, 2.0, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 1.0}, -1, 2.0, 1.0)
    assert observation.fresh_camera_frame({'measurement_stamp': 1.0}, -1, 2.0, 0.0)
    assert observation.fresh_camera_frame({'measurement_stamp': 2.0}, -1, 2.0, 0.0)
    assert not observation.fresh_camera_frame({'measurement_stamp': 1.0}, -1, 2.0, 0.0, timeout=0.5)


def test_image_publication_uses_capture_time_and_keeps_clock_independent():
    from pathlib import Path
    from types import SimpleNamespace
    from r2s2r.test_sim_clock import functions

    scope = functions(Path(__file__).with_name('run_sim.py'), {'on_render_step', 'mat3_to_quat'})
    events = []
    world = SimpleNamespace(current_time=2.3)
    data = {'rgb': np.ones((1, 1, 3), dtype=np.uint8), 'measurement_stamp': 2.25}
    intrinsics = {'fx': 700.0, 'fy': 700.0, 'width': 1600, 'height': 1200}
    scope.update(world=world, _frame_count=0, _last_pose_time=0., _pose_interval=.02,
        _last_pc_time=0., _pc_interval=.1, _last_img_time=0., _img_interval=.1,
        _last_imu_time=0., _imu_interval=.02, _ever_received_ros_cmd=False,
        _previous_pose_sample=None, _body_prim=SimpleNamespace(IsValid=lambda: False),
        _cmd=np.zeros(3), _last_scan_warning=0., _last_camera_stamps={}, _observation_epoch=0.,
        fresh_camera_frame=observation.fresh_camera_frame,
        suite=SimpleNamespace(get_data_by_type=lambda kind: {'camera1': data} if kind == 'camera' else {},
            sensor_to_body=lambda name: np.eye(4),
            camera_intrinsics=lambda name: intrinsics),
        bridge_server=SimpleNamespace(connected=False, get_reset_requested=lambda: None,
            get_latest_cmd_vel=lambda **_: None,
            send_clock=lambda stamp: events.append(('clock', stamp)),
            send_image=lambda rgb, **kw: events.append(('image', kw))), _img_quality=80)
    scope['on_render_step']()
    assert events == [('clock', 2.3), ('image', {'frame_id': 'camera1', 'quality': 80,
        'timestamp': 2.25, 'intrinsics': intrinsics,
        'body_to_camera': ([0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 1.0))})]
    for now, stamp in [(2.5, 2.25), (4.0, 2.25), (4.2, 4.3)]:
        world.current_time = now
        data['measurement_stamp'] = stamp
        scope['on_render_step']()
        assert events[-1] == ('clock', now)
    assert sum(event[0] == 'image' for event in events) == 1
    scope['_observation_epoch'] = 4.4
    scope['_last_camera_stamps'].clear()
    world.current_time = 4.6
    data['measurement_stamp'] = 4.4
    scope['on_render_step']()
    assert events[-1] == ('clock', 4.6)
    world.current_time = 4.8
    data['measurement_stamp'] = 4.7
    scope['on_render_step']()
    assert events[-1][1]['timestamp'] == 4.7


def test_robot_reset_discards_cached_camera_and_duplicate_state():
    from pathlib import Path
    from types import SimpleNamespace
    from r2s2r.test_sim_clock import functions

    scope = functions(Path(__file__).with_name('run_sim.py'), {'reset_robot'})
    writer = SimpleNamespace(latest={'measurement_stamp': 1.0})
    scope.update(world=SimpleNamespace(current_time=2.0), _previous_pose_sample=object(),
        _observation_epoch=0.0, _last_camera_stamps={'camera1': 1.0},
        suite=SimpleNamespace(_camera_writers={'camera1': writer}),
        robot=SimpleNamespace(robot=SimpleNamespace(get_world_pose=lambda: (None, None))))
    scope['reset_robot']()
    assert scope['_observation_epoch'] == 2.0
    assert not scope['_last_camera_stamps']
    assert not writer.latest


@pytest.mark.parametrize('initialize_fails', [False, True])
def test_main_closes_initialized_suite_before_application_even_on_setup_error(initialize_fails):
    from pathlib import Path
    from types import SimpleNamespace
    from r2s2r.test_sim_clock import functions

    scope = functions(Path(__file__).with_name('run_sim.py'), {'main'})
    events = []

    def initialize():
        events.append('initialize')
        if initialize_fails:
            raise RuntimeError('initialization failed')

    def play():
        raise RuntimeError('play failed')

    scope.update(world=SimpleNamespace(reset=lambda: None, play=play,
            add_physics_callback=lambda *args: None),
        robot=SimpleNamespace(initialize=lambda: None),
        suite=SimpleNamespace(initialize=initialize, close=lambda: events.append('suite_close')),
        bridge_server=SimpleNamespace(start=lambda: None, stop=lambda: events.append('bridge_stop')),
        simulation_app=SimpleNamespace(close=lambda: events.append('app_close')),
        _setup_keyboard=lambda: None, on_physics_step=lambda *args: None)
    with pytest.raises(RuntimeError):
        scope['main']()
    expected = ['initialize', 'bridge_stop']
    if not initialize_fails:
        expected.append('suite_close')
    assert events == expected + ['app_close']
