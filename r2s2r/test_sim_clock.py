"""Regression checks for simulation time across the Isaac/TCP/ROS boundary."""
import ast
import math
import runpy
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from r2s2r.bridge.protocol import MSG_CLOCK, encode_clock, encode_pose, encode_pointcloud
from r2s2r.bridge.tcp_server import TcpBridgeServer

ROOT = Path(__file__).resolve().parent.parent


def functions(path, names, class_name=None):
    nodes = ast.parse(path.read_text()).body
    if class_name:
        nodes = next(n for n in nodes if isinstance(n, ast.ClassDef) and n.name == class_name).body
    nodes = [n for n in nodes if isinstance(n, ast.FunctionDef) and n.name in names]
    scope = {'np': np, 'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), scope)
    return scope


class SimulationClockTest(unittest.TestCase):
    def test_clock_and_observation_wire_format(self):
        decode = runpy.run_path(str(ROOT/'ros1_ws/src/sim_bridge/src/protocol.py'))['decode_message']
        packets = [encode_clock(2.5), encode_pose(1,0,.3,0,0,0,1,timestamp=2.5),
                   encode_pointcloud(np.array([[1,0,0]]),timestamp=2.4)]
        buffer = b''.join(packets)
        for expected in [MSG_CLOCK, 1, 2]:
            kind, data, used = decode(buffer)
            self.assertEqual(kind, expected)
            self.assertIn(data['ts'], [2.5, 2.4])
            buffer = buffer[used:]
        self.assertFalse(buffer)

    def test_slow_simulation_uses_simulation_time_for_pose_and_velocity(self):
        scope = functions(ROOT/'r2s2r/run_sim.py', {'on_render_step', 'pose_twist', 'mat3_to_quat'})
        events = []; world = SimpleNamespace(current_time=1.)
        mat = np.eye(4)
        scope.update(world=world, _frame_count=0, _last_pose_time=0., _pose_interval=.02,
            _last_pc_time=0., _pc_interval=.1, _last_img_time=0., _img_interval=.1,
            _last_imu_time=0., _imu_interval=.02, _ever_received_ros_cmd=False,
            _previous_pose_sample=None, _body_prim=SimpleNamespace(IsValid=lambda:True),
            _cmd=np.zeros(3), _last_scan_warning=0., suite=None,
            get_robot_world_transform=lambda _:mat.copy(),
            bridge_server=SimpleNamespace(connected=False, get_reset_requested=lambda:None,
                get_latest_cmd_vel=lambda **_:None,
                send_clock=lambda t:events.append(('clock',t)),
                send_pose=lambda *p,**kw:events.append(('pose',p,kw))))
        scope['on_render_step']()
        world.current_time=1.02-5e-10; mat[0,3]=.75*(world.current_time-1.)
        # An arbitrarily long wall-time delay never enters the publication path.
        scope['on_render_step']()
        self.assertEqual(events[0], ('clock',1.))
        self.assertEqual(events[2], ('clock',world.current_time))
        self.assertAlmostEqual(events[-1][2]['timestamp'],world.current_time)
        self.assertAlmostEqual(events[-1][2]['vx'],.75)
        count=len(events)
        scope['on_render_step']()  # Paused clock: no new pose or velocity sample.
        self.assertEqual(len(events),count+1)
        self.assertEqual(events[-1],('clock',world.current_time))

    def test_bridge_bootstraps_clock_and_rejects_wall_time_observations(self):
        scope = functions(ROOT/'ros1_ws/src/sim_bridge/scripts/bridge_node.py', {'handle_message'}, 'BridgeNode')
        events=[]
        scope.update(MSG_CLOCK=6,MSG_POSE=1,MSG_POINTCLOUD=2,MSG_IMAGE=4,MSG_IMU=5,
            Clock=lambda **kw:kw,rospy=SimpleNamespace(Time=SimpleNamespace(from_sec=lambda t:t),
                logwarn_throttle=lambda *args:None))
        bridge=SimpleNamespace(use_sim_time=True,_sim_time=None,_last_cloud_stamp=0.,
            clock_pub=SimpleNamespace(publish=lambda x:events.append(('clock',x))),
            publish_odometry=lambda x:events.append(('pose',x)))
        handle=scope['handle_message']
        handle(bridge,1,{'ts':1000000.})
        self.assertFalse(events)
        handle(bridge,6,{'ts':2.})
        handle(bridge,1,{'ts':2.})
        handle(bridge,1,{'ts':1000000.})
        self.assertEqual([x[0] for x in events],['clock','pose'])
        handle(bridge,6,{'ts':float('nan')})
        self.assertEqual(bridge._sim_time,2.)

    def test_cloud_age_uses_received_clock_without_rospy_subscription_race(self):
        scope = functions(ROOT/'ros1_ws/src/sim_bridge/scripts/bridge_node.py', {'publish_pointcloud'}, 'BridgeNode')
        events=[]
        class Field:
            FLOAT32=7
            def __init__(self,*args): pass
        scope.update(PointField=Field,rospy=SimpleNamespace(Header=SimpleNamespace,
            Time=SimpleNamespace(from_sec=lambda t:t,now=lambda:None)),
            pc2=SimpleNamespace(create_cloud=lambda h,f,p:h))
        bridge=SimpleNamespace(use_sim_time=True,_sim_time=2.,_last_cloud_stamp=0.,
            pc_pub=SimpleNamespace(publish=lambda x:events.append(x)))
        publish=scope['publish_pointcloud']
        publish(bridge,{'ts':1.9,'xyz':np.ones((1,3))})
        publish(bridge,{'ts':1.9,'xyz':np.ones((1,3))})
        publish(bridge,{'ts':2.1,'xyz':np.ones((1,3))})
        self.assertEqual(len(events),1)
        self.assertEqual(events[0].stamp,1.9)

    def test_clock_and_sensor_queue_drains_with_partial_writes(self):
        output=bytearray()
        class Socket:
            blocked=False
            def send(self, data):
                if output and not self.blocked:
                    self.blocked=True
                    raise BlockingIOError()
                n=min(len(data),7); output.extend(data[:n]); return n
        server=TcpBridgeServer()
        server._client_sock=Socket()
        server.send_clock(2.)
        server.send_pose(1,0,0,0,0,0,1,timestamp=2.)
        expected=b''.join(server._send_queue)
        server._flush_send_queue()
        self.assertEqual(bytes(output),expected[:7])
        server._flush_send_queue()
        self.assertEqual(bytes(output),expected)
        self.assertFalse(server._write_buf or server._send_queue)


if __name__=='__main__':
    unittest.main()
