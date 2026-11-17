"""Keyboard yields /cmd_vel to automatic publishers, including on shutdown."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class TeleopOwnershipTest(unittest.TestCase):
    def run_keyboard(self, publishers):
        sent = []
        keys = iter(['', '\x03'])
        rospy = SimpleNamespace(
            init_node=lambda *a: None, get_name=lambda: '/teleop_keyboard',
            resolve_name=lambda name: name,
            get_master=lambda: SimpleNamespace(getSystemState=lambda: (1, '', [publishers, [], []])),
            get_param=lambda name, default: default,
            Publisher=lambda *a, **kw: SimpleNamespace(publish=sent.append),
            Rate=lambda rate: SimpleNamespace(sleep=lambda: None),
            is_shutdown=lambda: False, logwarn_throttle=lambda *a: None,
            ROSInterruptException=RuntimeError)
        path = Path(__file__).with_name('teleop_keyboard.py')
        nodes = ast.parse(path.read_text()).body
        scope = dict(rospy=rospy, HELP_TEXT='', BINDINGS={},
                     get_key_nonblocking=lambda **kw: next(keys),
                     Twist=lambda: SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace()))
        exec(compile(ast.Module(body=[n for n in nodes if isinstance(n, ast.FunctionDef)
                                    and n.name in ('main', 'other_velocity_publisher')],
                                type_ignores=[]), str(path), 'exec'), scope)
        scope['main']()
        return sent

    def test_automatic_publisher_suppresses_idle_and_exit_zero(self):
        self.assertEqual(self.run_keyboard([('/cmd_vel', ['/path_tracker', '/teleop_keyboard'])]), [])

    def test_keyboard_alone_retains_stop_commands(self):
        self.assertEqual(len(self.run_keyboard([('/cmd_vel', ['/teleop_keyboard'])])), 2)

    def test_unrelated_publisher_does_not_disable_keyboard(self):
        self.assertEqual(len(self.run_keyboard([('/scan', ['/lidar'])])), 2)


if __name__ == '__main__':
    unittest.main()
