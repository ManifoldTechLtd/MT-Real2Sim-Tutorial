import math

import numpy as np
import pytest

from frame_math import compose_pose, invert_pose


def test_invert_identity():
    xyz, quat = invert_pose([0, 0, 0], [0, 0, 0, 1])
    assert np.allclose(xyz, [0, 0, 0])
    assert np.allclose(quat, [0, 0, 0, 1])


def test_invert_translation_only():
    xyz, quat = invert_pose([0.3, -0.02, 0.09], [0, 0, 0, 1])
    assert np.allclose(xyz, [-0.3, 0.02, -0.09])
    assert np.allclose(quat, [0, 0, 0, 1])


def test_compose_identity_body():
    xyz, quat = compose_pose([0, 0, 0], [0, 0, 0, 1],
                             [0.313, -0.025, 0.086], [0, 0, 0, 1])
    assert np.allclose(xyz, [0.313, -0.025, 0.086])
    assert np.allclose(quat, [0, 0, 0, 1])


def test_compose_yaw_90():
    half = math.pi / 4
    xyz, quat = compose_pose([1, 2, 0], [0, 0, math.sin(half), math.cos(half)],
                             [1, 0, 0], [0, 0, 0, 1])
    assert np.allclose(xyz, [1, 3, 0], atol=1e-9)
    assert np.allclose(quat, [0, 0, math.sin(half), math.cos(half)], atol=1e-9)


def test_compose_then_invert_roundtrip():
    body = ([1.2, -0.4, 0.32], [0.01, -0.02, 0.03, 0.999])
    sensor = ([0.313, -0.025, 0.086], [-0.008, -0.002, -0.005, 0.99995])
    p, q = compose_pose(*body, *sensor)
    p2, q2 = compose_pose(p, q, *invert_pose(sensor[0], sensor[1]))
    assert np.allclose(p2, body[0], atol=1e-9)
    norm = np.linalg.norm(body[1])
    assert np.allclose(np.abs(np.dot(q2, np.array(body[1]) / norm)), 1.0, atol=1e-9)
