import ast
import math
from pathlib import Path
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parent

def load_functions(path, names, class_name=None):
    tree = ast.parse(path.read_text())
    nodes = tree.body
    if class_name:
        nodes = next((n for n in nodes if isinstance(n, ast.ClassDef) and n.name == class_name)).body
    selected = [n for n in nodes if isinstance(n, ast.FunctionDef) and n.name in names]
    for node in selected:
        node.decorator_list = []
    scope = {'np': np, 'math': math}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), scope)
    return scope

def transform(yaw=0.0, pitch=0.0, roll=0.0, position=(0.0, 0.0, 0.0)):
    c, s = (math.cos, math.sin)
    rz = np.array([[c(yaw), -s(yaw), 0], [s(yaw), c(yaw), 0], [0, 0, 1]])
    ry = np.array([[c(pitch), 0, s(pitch)], [0, 1, 0], [-s(pitch), 0, c(pitch)]])
    rx = np.array([[1, 0, 0], [0, c(roll), -s(roll)], [0, s(roll), c(roll)]])
    mat = np.eye(4)
    mat[:3, :3] = rz @ ry @ rx
    mat[:3, 3] = position
    return mat

class PoseFeedbackTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.functions = load_functions(ROOT / 'run_sim.py', {'pose_twist', 'mat3_to_quat'})

    def twist(self, previous, mat, stamp):
        return self.functions['pose_twist'](previous, mat, stamp)

    def test_blocked_pose_reports_zero(self):
        mat = transform(yaw=2.0, pitch=0.4, roll=0.2)
        linear, angular = self.twist((mat, 10.0), mat, 10.2)
        np.testing.assert_allclose(linear, 0.0, atol=1e-12)
        np.testing.assert_allclose(angular, 0.0, atol=1e-12)

    def test_body_frame_and_publication_time(self):
        old = transform(yaw=1.0, pitch=0.4, roll=-0.2)
        new = old.copy()
        expected = np.array([0.6, -0.2, 0.1])
        new[:3, 3] += new[:3, :3] @ expected * 0.25
        linear, angular = self.twist((old, 20.0), new, 20.25)
        np.testing.assert_allclose(linear, expected, atol=1e-12)
        np.testing.assert_allclose(angular, 0.0, atol=1e-12)

    def test_yaw_wrap(self):
        old = transform(yaw=math.pi - 0.01)
        new = transform(yaw=-math.pi + 0.01)
        _, angular = self.twist((old, 1.0), new, 1.1)
        np.testing.assert_allclose(angular, [0.0, 0.0, 0.2], atol=1e-12)

    def test_pitch_rotation_reports_body_angular_velocity(self):
        old = transform(yaw=2.0, pitch=0.3)
        new = transform(yaw=2.0, pitch=0.32)
        _, angular = self.twist((old, 1.0), new, 1.1)
        np.testing.assert_allclose(angular, [0.0, 0.2, 0.0], atol=1e-12)

    def test_initial_reset_and_nonincreasing_time(self):
        mat = transform(position=(100.0, 20.0, -4.0))
        for previous, stamp in [(None, 1.0), ((np.eye(4), 1.0), 1.0), ((np.eye(4), 2.0), 1.0)]:
            with self.subTest(previous=previous, stamp=stamp):
                linear, angular = self.twist(previous, mat, stamp)
                np.testing.assert_array_equal(linear, np.zeros(3))
                np.testing.assert_array_equal(angular, np.zeros(3))
