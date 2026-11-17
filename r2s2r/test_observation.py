"""Regression checks for render-time observations and the ROS bridge contract."""
import ast
import runpy
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from r2s2r.core.observation import decode_lidar_frame, filter_body_returns, fresh_scan, rigid_transform
from r2s2r.bridge.protocol import encode_pointcloud

ROOT = Path(__file__).resolve().parent.parent


def frame():
    pose = np.eye(4); pose[:3,3] = [.35,0,.45]
    return {'IsaacReadSimulationTime': {'simulationTime': 100.0},
            'IsaacCreateRTXLidarScanBuffer': {
                'data': np.array([[2.95,0,.5],[0,0,0],[1,0,float('nan')]],dtype=np.float32),
                'timestamp': np.array([1_000_000_000]*3,dtype=np.uint64),
                'distance': np.array([2.6,-1,1]), 'info': {'transform': pose.T}}}


class ObservationTest(unittest.TestCase):
    def test_disabled_camera_is_not_created_other_sensors_remain(self):
        tree=ast.parse((ROOT/'r2s2r/core/sensor_suite.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SensorSuite')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='build')
        scope={'_log':SimpleNamespace(info=lambda *a:None,error=lambda *a,**kw:None)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'sensor_build','exec'),scope)
        created=[]
        configs={name:{'type':kind,'raw_cfg':raw,'translation':[], 'rotation':[]}
                 for name,kind,raw in [('camera1','camera',{'enabled':False}),
                                       ('lidar1','lidar',{}),('imu_center','imu',{})]}
        def create(name):
            created.append(name)
            return object()
        suite=SimpleNamespace(_sensor_configs=configs,_sensors={},_use_rimu=False,
            _create_camera=lambda world,name,cfg:self.fail('disabled camera was created'),
            _create_lidar=lambda world,name,cfg:create(name),
            _create_imu=lambda world,name,cfg:create(name))
        scope['build'](suite,None)
        self.assertEqual(created,['lidar1','imu_center'])
        self.assertEqual(set(suite._sensors),{'lidar1','imu_center'})

    def test_measurement_time_pose_and_owned_points(self):
        source = frame()
        obs = decode_lidar_frame(source)
        self.assertEqual(obs['measurement_stamp'],100.0)
        np.testing.assert_allclose(obs['lidar_to_world'][:3,3],[.35,0,.45])
        self.assertEqual(len(obs['point_cloud']),1)
        source['IsaacCreateRTXLidarScanBuffer']['data'][:] = 0
        np.testing.assert_allclose(obs['point_cloud'],[[2.95,0,.5]])

    def test_duplicate_stale_future_and_pre_reset_observations_rejected(self):
        obs = decode_lidar_frame(frame())
        self.assertTrue(fresh_scan(obs,0,100.2,99.))
        self.assertFalse(fresh_scan(obs,obs['scan_stamp_ns'],100.2,99.))
        self.assertFalse(fresh_scan(obs,0,102.,99.))
        self.assertFalse(fresh_scan(obs,0,99.,98.))
        self.assertFalse(fresh_scan(obs,0,100.2,100.1))

    def test_scan_timestamp_buffer_is_not_assumed_point_aligned(self):
        source = frame()
        source['IsaacCreateRTXLidarScanBuffer']['timestamp'] = np.array([1_000_000_000], dtype=np.uint64)
        obs = decode_lidar_frame(source)
        self.assertEqual(obs['scan_stamp_ns'], 1_000_000_000)
        np.testing.assert_allclose(obs['point_cloud'], [[2.95,0,.5]])

    def test_missing_time_mixed_scan_and_bad_pose_rejected(self):
        for change in ['time','span','pose','distance','zero_timestamp','float_timestamp']:
            source = frame()
            scan = source['IsaacCreateRTXLidarScanBuffer']
            if change == 'time': source.pop('IsaacReadSimulationTime')
            if change == 'span':
                scan['distance'][1] = 1
                scan['timestamp'][1] += 20_000_000
            if change == 'pose': scan['info']['transform'] = np.zeros((4,4))
            if change == 'distance': scan.pop('distance')
            if change == 'zero_timestamp': scan['timestamp'][0] = 0
            if change == 'float_timestamp': scan['timestamp'] = scan['timestamp'].astype(float)
            with self.subTest(change=change), self.assertRaises((ValueError,KeyError)):
                decode_lidar_frame(source)

    def test_body_filter_preserves_ground_and_nearby_obstacle_after_rotation(self):
        body_lidar = rigid_transform([.35,0,.2],[1,0,0,0])
        body_world = rigid_transform([2,3,.25],[2**-.5,0,0,2**-.5])
        points_body = np.array([[0,0,0],[.35,0,.2],[0,0,-.25],[.5,0,0]])
        points_world = points_body @ body_world[:3,:3].T + body_world[:3,3]
        kept = filter_body_returns(points_world,body_world@body_lidar,body_lidar,[.7,.31,.4])
        np.testing.assert_allclose(kept,points_world[2:])

    def test_wire_format_keeps_measurement_pose_and_time(self):
        payload = encode_pointcloud(np.array([[2.95,0,.5]]),'map',100.,[0,0,.25,0,0,0,1])
        decode = runpy.run_path(str(ROOT/'ros1_ws/src/sim_bridge/src/protocol.py'))['decode_message']
        message_type, data, consumed = decode(payload)
        self.assertEqual(message_type,2)
        self.assertEqual(consumed,len(payload))
        self.assertEqual(data['ts'],100.)
        self.assertEqual(data['body_pose'],[0,0,.25,0,0,0,1])
        np.testing.assert_allclose(data['xyz'],[[2.95,0,.5]])

    def test_double_cylinder_filter_preserves_empty_bounding_box_corners(self):
        body_world = rigid_transform([2,3,.30],[2**-.5,0,0,2**-.5])
        body_lidar = rigid_transform([.43,0,.10],[1,0,0,0])
        local = np.array([[.18,0,0],[-.18,0,.1],[.4,.23,0], [0,0,-.3],[.5,0,0]])
        world = local @ body_world[:3,:3].T + body_world[:3,3]
        kept = filter_body_returns(world,body_world@body_lidar,body_lidar,[.86,.5,.2],
            body_shape='double_cylinder',cylinder_radius=.25,cylinder_offset=.18)
        np.testing.assert_allclose(kept,world[2:])

    def test_bridge_inserts_measurement_tf_and_rejects_duplicate_cloud(self):
        tree=ast.parse((ROOT/'ros1_ws/src/sim_bridge/scripts/bridge_node.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_pointcloud')
        events=[]
        class Field:
            FLOAT32=7
            def __init__(self,*args): pass
        scope={'np':np,'PointField':Field,
               'rospy':SimpleNamespace(Header=SimpleNamespace,
                    Time=SimpleNamespace(from_sec=lambda t:t,now=lambda:SimpleNamespace(to_sec=lambda:100.2)),
                    logwarn_throttle=lambda *args:None),
               'pc2':SimpleNamespace(create_cloud=lambda header,fields,xyz:(header,xyz))}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'bridge_method','exec'),scope)
        bridge=SimpleNamespace(_last_cloud_stamp=0.,_odom_tf_stamps=(),use_sim_time=False,map_frame='map',odom_frame='odom',base_frame='base_link',
            tf_broadcaster=SimpleNamespace(sendTransform=lambda *args:events.append(('tf',args))),
            pc_pub=SimpleNamespace(publish=lambda msg:events.append(('cloud',msg))))
        data={'xyz':np.array([[1,0,0]]),'ts':100.,'frame_id':'map','body_pose':[2,3,.25,0,0,0,1]}
        scope['publish_pointcloud'](bridge,data)
        scope['publish_pointcloud'](bridge,data)
        self.assertEqual([event[0] for event in events],['tf','cloud'])
        self.assertEqual(events[0][1],((2.,3.,.25),(0.,0.,0.,1.),100.,'base_link','odom'))
        self.assertEqual(events[1][1][0].stamp,100.)
        events.clear()
        bridge._last_cloud_stamp=0.
        bridge._odom_tf_stamps=(100.,100.1)
        scope['publish_pointcloud'](bridge,data)
        self.assertEqual([event[0] for event in events],['cloud'])


if __name__=='__main__':
    unittest.main()
