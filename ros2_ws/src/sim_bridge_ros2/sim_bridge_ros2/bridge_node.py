#!/usr/bin/env python3
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
"""
sim_bridge_ros2 — Isaac Sim TCP(9090) ↔ ROS2 桥

移植自 MT-Real2Sim-Tutorial/ros1_ws/src/sim_bridge/scripts/bridge_node.py（rospy 版），
话题/TF/QoS 语义保持一致，使 ROS1 与 ROS2 两条线可共用同一套 Isaac 仿真：

  发布: /state_estimation        (nav_msgs/Odometry)
        /registered_scan         (sensor_msgs/PointCloud2, 世界系点云)
        /camera/image/compressed (sensor_msgs/CompressedImage)
        /imu/data                (sensor_msgs/Imu)
        /odin1/* aliases         (ODometry/cloud/image/imu, publish_odin1_aliases=true)
        TF: map→odom (静态 identity) / odom→base_link (动态, 来自 POSE)
            frame_layout=odin 时改为 odom→imu（传感器位姿），
            传感器静态边由导航栈 launch 发布（与真机一致）。
  订阅: /cmd_vel   (geometry_msgs/Twist) → TCP CMD_VEL
        /sim_reset (std_msgs/Empty)      → TCP RESET(to_start=True)

线程模型: 主线程 rclpy.spin()；后台线程 200Hz 轮询 socket（与 ROS1 版一致）。
"""

import io
import json
import math
import os
import socket
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
from std_msgs.msg import Empty, String
from rosgraph_msgs.msg import Clock as ClockMsg
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .camera_projection import (colorize_points, compose_transform,
                                nearest_stamp_index)
from .impairment import (CloudImpair, CmdImpair, ImuImpair, OdomImpair,
                         make_profile)
from .frame_math import compose_pose
from .protocol import (MSG_CLOCK, MSG_IMAGE, MSG_IMU, MSG_POINTCLOUD, MSG_POSE,
                       decode_message, encode_cmd_vel, encode_reset,
                       encode_set_pose)


def _sensor_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class SimBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        # ── 参数（与 ROS1 版 ~param 一一对应）──
        self.declare_parameter('tcp_host', 'localhost')
        self.declare_parameter('tcp_port', 9090)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('poll_rate_hz', 200.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('footprint_frame', 'base_footprint')
        self.declare_parameter('sensor_base_frame', 'odin1_base_link')
        self.declare_parameter('lidar_frame', 'lidar1')
        self.declare_parameter('camera_frame', 'camera1')
        self.declare_parameter('imu_frame', 'imu_link')

        self.declare_parameter('base_height', 0.20)
        self.declare_parameter('sensor_base_offset_xyz', [0.0, 0.0, 0.30])
        self.declare_parameter('sensor_base_rotation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('camera_offset_xyz', [0.3, 0.0, 0.10])
        self.declare_parameter('imu_offset_xyz', [0.0, 0.0, 0.0])

        # ONS2 integration: odin layout publishes odom->imu (sensor pose) and
        # lets the navigation launch own imu->odin1_base_link->base_link.
        self.declare_parameter('frame_layout', 'legacy')
        self.declare_parameter('publish_odin1_aliases', True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('color_lut_path', '')
        self.declare_parameter('lut_gamma', 1.0)
        self.declare_parameter('camera_color', False)
        self.declare_parameter('camera_color_max_age', 0.25)

        g = self.get_parameter
        self.host = g('tcp_host').value
        self.port = int(g('tcp_port').value)
        self.reconnect_interval = float(g('reconnect_interval').value)
        self.poll_period = 1.0 / max(float(g('poll_rate_hz').value), 1.0)

        self.map_frame = g('map_frame').value
        self.odom_frame = g('odom_frame').value
        self.base_frame = g('base_frame').value
        self.footprint_frame = g('footprint_frame').value
        self.sensor_base_frame = g('sensor_base_frame').value
        self.lidar_frame = g('lidar_frame').value
        self.camera_frame = g('camera_frame').value
        self.imu_frame = g('imu_frame').value

        self.base_height = float(g('base_height').value)
        self.sensor_base_offset = [float(v) for v in g('sensor_base_offset_xyz').value]
        self.sensor_base_rotation = [float(v) for v in g('sensor_base_rotation_xyzw').value]
        self.camera_offset = [float(v) for v in g('camera_offset_xyz').value]
        self.imu_offset = [float(v) for v in g('imu_offset_xyz').value]
        self.publish_image = bool(g('publish_image').value)
        self.camera_color = bool(g('camera_color').value)
        self.camera_color_max_age = float(g('camera_color_max_age').value)
        self._pose_hist = deque(maxlen=400)
        self._img_hist = deque(maxlen=8)
        self._decoded_image = None
        lut_path = str(g('color_lut_path').value).strip()
        self._lut_keys = None
        self._lut_colors = None
        self._lut_voxel = 0.0
        if lut_path and os.path.isfile(lut_path):
            data = np.load(lut_path)
            self._lut_keys = data['keys'].astype(np.int64)
            self._lut_colors = data['colors'].astype(np.float32)
            gamma = float(g('lut_gamma').value)
            if gamma > 0 and abs(gamma - 1.0) > 1e-6:
                # Source PCD colors are ~2x darker than camera samples on the same
                # points; gamma 0.5 matched their medians (61 -> 125 luminance).
                self._lut_colors = 255.0 * np.power(self._lut_colors / 255.0, gamma)
            self._lut_colors = np.clip(self._lut_colors, 0, 255).astype(np.uint8)
            self._lut_voxel = float(data['voxel'])
            self.get_logger().info(
                f'[sim_bridge_ros2] color LUT: {len(self._lut_keys)} voxels '
                f'voxel={self._lut_voxel} gamma={gamma:.2f} from {lut_path}')
        if self.camera_color:
            self.get_logger().info(
                '[sim_bridge_ros2] camera color: projecting the nearest image '
                f'(max age {self.camera_color_max_age:.2f}s) onto cloud_slam')
        self.frame_layout = str(g('frame_layout').value).strip().lower()
        self.publish_odin1_aliases = bool(g('publish_odin1_aliases').value)
        if self.frame_layout not in ('legacy', 'odin'):
            raise ValueError(f'unknown frame_layout: {self.frame_layout}')

        # ── s2r 差距注入（默认关闭；关闭时不实例化任何注入对象）──
        self.impair_profile = 'off'
        self.cmd_impair = None
        self.odom_impair = None
        self.cloud_impair = None
        self.imu_impair = None
        self._last_cmd_sent = None
        self._last_applied_pub = 0.0
        self._last_stats_pub = 0.0
        self._impair_debug = False
        self._impair_seed = 0
        self._odom_rigid_cloud = False
        if bool(self.declare_parameter('impair.enable', False).value):
            self.impair_profile = str(
                self.declare_parameter('impair.profile', 'real').value)
            seed = int(self.declare_parameter('impair.seed', 1).value)
            self._impair_seed = seed
            self._impair_debug = bool(
                self.declare_parameter('impair.debug_topics', True).value)
            cfgs = make_profile(self.impair_profile)
            for channel, cfg in cfgs.items():
                self._override_cfg(f'impair.{channel}', cfg)
            self.cmd_impair = CmdImpair(cfgs['cmd'], seed) if cfgs['cmd'].enable else None
            self.odom_impair = OdomImpair(cfgs['odom'], seed + 10) if cfgs['odom'].enable else None
            self.cloud_impair = CloudImpair(cfgs['cloud'], seed + 20) if cfgs['cloud'].enable else None
            self.imu_impair = ImuImpair(cfgs['imu'], seed + 30) if cfgs['imu'].enable else None
            self._odom_rigid_cloud = bool(cfgs['odom'].rigid_cloud)

        # ── 发布者 ──
        self.clock_pub = self.create_publisher(ClockMsg, '/clock', 10)
        self.odom_pub = self.create_publisher(Odometry, '/state_estimation', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/registered_scan', _sensor_qos(5))
        self.img_pub = self.create_publisher(CompressedImage, '/camera/image/compressed',
                                             _sensor_qos(5))
        self.imu_pub = self.create_publisher(Imu, '/imu/data', _sensor_qos(50))
        self.odin_odom_pub = None
        self.odin_cloud_pub = None
        self.odin_img_pub = None
        self.odin_imu_pub = None
        if self.publish_odin1_aliases:
            self.odin_odom_pub = self.create_publisher(Odometry, '/odin1/odometry', 10)
            self.odin_cloud_pub = self.create_publisher(
                PointCloud2, '/odin1/cloud_slam', _sensor_qos(5))
            self.odin_img_pub = self.create_publisher(
                CompressedImage, '/odin1/image/compressed', _sensor_qos(5))
            self.odin_imu_pub = self.create_publisher(Imu, '/odin1/imu', _sensor_qos(50))
        self.gt_pub = None
        self.applied_pub = None
        self.impair_stats_pub = None
        if self._impair_debug:
            self.gt_pub = self.create_publisher(Odometry, '/sim_bridge/ground_truth', 10)
            self.applied_pub = self.create_publisher(Twist, '/sim_bridge/applied_cmd', 10)
            self.impair_stats_pub = self.create_publisher(
                String, '/sim_bridge/impair_stats', 10)

        # ── TF ──
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tfs()

        # ── 订阅者 ──
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 1)
        self.create_subscription(Empty, '/sim_reset', self._reset_cb, 1)
        # 自动化测试用：把机器人传送到任意 (x, y, yaw)，z 由仿真侧 raycast 找地面。
        # orientation 只需 yaw（仿真侧只写 yaw，pitch/roll 由地形拟合得出）。
        self.create_subscription(PoseStamped, '/sim_set_pose', self._set_pose_cb, 1)

        # ── TCP 状态 ──
        self.sock = None
        self.recv_buffer = bytearray()
        self._send_buffer = b""
        self.connected = False
        self._last_rx = 0.0
        self._lock = threading.Lock()
        self._latest_cmd = None
        self._reset_requested = False
        self._set_pose_req = None
        self._stop = threading.Event()

        # 统计（便于自动化验收断言）
        self.stats = {'pose': 0, 'cloud': 0, 'imu': 0, 'image': 0, 'cmd_sent': 0}


        self.get_logger().info(
            f'[sim_bridge_ros2] target={self.host}:{self.port} layout={self.frame_layout} '
            f'TF={self.map_frame}->{self.odom_frame}->'
            f'{self.imu_frame if self.frame_layout == "odin" else self.base_frame} '
            f'sensor_base={self.sensor_base_frame}@{self.sensor_base_offset} '
            f'base_height={self.base_height}')
        if self.impair_profile != 'off':
            self.get_logger().info(
                f'[sim_bridge_ros2] impair profile={self.impair_profile} '
                f'seed={self._impair_seed} debug={self._impair_debug}')

    def _override_cfg(self, prefix, cfg):
        """profile 的每个字段都可被 impair.<channel>.<field> 覆盖。"""
        for name, default in vars(cfg).items():
            key = f'{prefix}.{name}'
            if isinstance(default, bool):
                setattr(cfg, name, bool(self.declare_parameter(key, default).value))
            else:
                setattr(cfg, name, float(self.declare_parameter(key, default).value))

    # ────────────────────────── 工具 ──────────────────────────
    @staticmethod
    def _stamp(ts):
        try:
            return Time(seconds=float(ts)).to_msg()
        except Exception:
            return Time().to_msg()

    def _static_tf(self, parent, child, xyz, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.w = 1.0
        return t

    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()
        tfs = [
            self._static_tf(self.map_frame, self.odom_frame, [0, 0, 0], now),
            self._static_tf(self.base_frame, self.footprint_frame,
                            [0, 0, -self.base_height], now),
        ]
        if self.frame_layout != 'odin':
            tfs.extend([
                self._static_tf(self.base_frame, self.sensor_base_frame,
                                self.sensor_base_offset, now),
                self._static_tf(self.sensor_base_frame, self.lidar_frame, [0, 0, 0], now),
                self._static_tf(self.base_frame, self.camera_frame, self.camera_offset, now),
                self._static_tf(self.base_frame, self.imu_frame, self.imu_offset, now),
            ])
        self.static_tf_broadcaster.sendTransform(tfs)

    # ────────────────────────── 回调 ──────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._latest_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.get_logger().info(
            f'[sim_bridge_ros2] /cmd_vel vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} '
            f'wz={msg.angular.z:.2f}', throttle_duration_sec=2.0)

    def _reset_cb(self, _msg):
        with self._lock:
            self._reset_requested = True
        self.get_logger().info('[sim_bridge_ros2] reset requested (to_start=True)')

    def _set_pose_cb(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        z = float(p.z) if abs(float(p.z)) > 1e-6 else None
        with self._lock:
            self._set_pose_req = (p.x, p.y, yaw, z)
        self.get_logger().info(
            f'[sim_bridge_ros2] set_pose requested x={p.x:.2f} y={p.y:.2f} '
            f'z={z if z is not None else "raycast"} yaw={math.degrees(yaw):.1f}deg')

    # ────────────────────────── TCP ──────────────────────────
    def _connect(self) -> bool:
        if self.sock is not None:
            self._disconnect()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self.host, self.port))
            s.setblocking(False)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = s
            self.connected = True
            self.recv_buffer.clear()
            self._last_rx = time.monotonic()
            self.get_logger().info(f'[sim_bridge_ros2] connected to {self.host}:{self.port}')
            return True
        except (socket.error, OSError):
            self.sock = None
            self.connected = False
            return False

    def _disconnect(self):
        self._send_buffer = b""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.connected = False
        self.recv_buffer.clear()

    def _send(self, payload: bytes):
        """Queue a message and flush without ever blocking the receive loop."""
        self._send_buffer += payload
        return self._flush_send_buffer()

    def _flush_send_buffer(self):
        if self.sock is None:
            self._send_buffer = b""
            return False
        while self._send_buffer:
            try:
                sent = self.sock.send(self._send_buffer)
            except BlockingIOError:
                return True  # keep the remainder for the next loop iteration
            except (ConnectionResetError, BrokenPipeError, OSError):
                self._send_buffer = b""
                self._disconnect()
                return False
            if sent <= 0:
                return True
            self._send_buffer = self._send_buffer[sent:]
        return True

    def socket_loop(self):
        """后台线程：200Hz 轮询收数据 + 转发 cmd_vel/reset。"""
        while not self._stop.is_set() and rclpy.ok():
            if not self.connected:
                if not self._connect():
                    self.get_logger().warn(
                        '[sim_bridge_ros2] waiting for Isaac TCP server...',
                        throttle_duration_sec=5.0)
                    self._stop.wait(self.reconnect_interval)
                    continue

            # 收
            try:
                data = self.sock.recv(65536)
                if data:
                    self._last_rx = time.monotonic()
                if not data:
                    self.get_logger().warn('[sim_bridge_ros2] server disconnected')
                    self._disconnect()
                    continue
                self.recv_buffer.extend(data)
            except BlockingIOError:
                pass
            except (ConnectionResetError, BrokenPipeError, OSError):
                self.get_logger().warn('[sim_bridge_ros2] connection lost')
                self._disconnect()
                continue

            if self.connected and time.monotonic() - self._last_rx > 5.0:
                self.get_logger().warn(
                    '[sim_bridge_ros2] no data for 5s; reconnecting')
                self._disconnect()
                continue
            while True:
                msg_type, payload, consumed = decode_message(bytes(self.recv_buffer))
                if msg_type is None:
                    break
                self.recv_buffer = self.recv_buffer[consumed:]
                self._handle(msg_type, payload)
            self._flush_send_buffer()

            # 发
            now = time.monotonic()
            with self._lock:
                cmd = self._latest_cmd
                self._latest_cmd = None
                reset_req = self._reset_requested
                self._reset_requested = False
                set_pose = self._set_pose_req
                self._set_pose_req = None
            if self.cmd_impair is not None:
                applied = self.cmd_impair.update(cmd, now)
                if self.connected and self._cmd_changed(applied):
                    if self._send(encode_cmd_vel(*applied)):
                        self.stats['cmd_sent'] += 1
                self._publish_applied(applied, now)
            elif cmd is not None and self.connected:
                if self._send(encode_cmd_vel(*cmd)):
                    self.stats['cmd_sent'] += 1
            if reset_req and self.connected:
                self._send(encode_reset(to_start=True))
            if set_pose is not None and self.connected:
                self._send(encode_set_pose(*set_pose))
            if self.odom_impair is not None:
                for item in self.odom_impair.flush(now):
                    self._publish_odom_msg(item)
            if self.cloud_impair is not None:
                for item in self.cloud_impair.flush(now):
                    self._publish_cloud_msg(item)
            self._publish_impair_stats(now)

            self._stop.wait(self.poll_period)

    def _cmd_changed(self, applied, eps=1e-3):
        if self._last_cmd_sent is None or any(
                abs(a - b) > eps for a, b in zip(applied, self._last_cmd_sent)):
            self._last_cmd_sent = tuple(applied)
            return True
        return False

    def _publish_applied(self, applied, now):
        if self.applied_pub is None or now - self._last_applied_pub < 0.02:
            return
        self._last_applied_pub = now
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = applied
        self.applied_pub.publish(msg)

    def _publish_impair_stats(self, now):
        if self.impair_stats_pub is None or now - self._last_stats_pub < 1.0:
            return
        self._last_stats_pub = now
        payload = {
            'profile': self.impair_profile,
            'seed': self._impair_seed,
            'cloud': self.cloud_impair.stats() if self.cloud_impair else None,
            'odom_queue': self.odom_impair.queue_len() if self.odom_impair else 0,
            'cmd_queue': self.cmd_impair.queue_len() if self.cmd_impair else 0,
        }
        self.impair_stats_pub.publish(String(data=json.dumps(payload)))

    # ────────────────────────── 消息处理 ──────────────────────────
    def _handle(self, msg_type, payload):
        if msg_type == MSG_CLOCK:
            self._publish_clock(payload)
        elif msg_type == MSG_POSE:
            self._publish_odometry(payload)
        elif msg_type == MSG_POINTCLOUD:
            self._publish_pointcloud(payload)
        elif msg_type == MSG_IMAGE:
            self._publish_image(payload)
        elif msg_type == MSG_IMU:
            self._publish_imu(payload)

    def _publish_clock(self, d):
        self.clock_pub.publish(ClockMsg(clock=self._stamp(d['ts'])))

    def _make_odom(self, d, child_frame=None, pose=None):
        odom = Odometry()
        odom.header.stamp = self._stamp(d['ts'])
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = child_frame or self.base_frame

        p = odom.pose.pose
        xyz, quat = pose if pose is not None else (
            (d['x'], d['y'], d['z']), (d['qx'], d['qy'], d['qz'], d['qw']))
        p.position.x = float(xyz[0])
        p.position.y = float(xyz[1])
        p.position.z = float(xyz[2])
        p.orientation.x = float(quat[0])
        p.orientation.y = float(quat[1])
        p.orientation.z = float(quat[2])
        p.orientation.w = float(quat[3])

        odom.twist.twist.linear.x = float(d.get('vx', 0.0))
        odom.twist.twist.linear.y = float(d.get('vy', 0.0))
        odom.twist.twist.linear.z = float(d.get('vz', 0.0))
        odom.twist.twist.angular.x = float(d.get('wx', 0.0))
        odom.twist.twist.angular.y = float(d.get('wy', 0.0))
        odom.twist.twist.angular.z = float(d.get('wz', 0.0))
        return odom

    def _sensor_pose(self, d):
        return compose_pose([d['x'], d['y'], d['z']],
                            [d['qx'], d['qy'], d['qz'], d['qw']],
                            self.sensor_base_offset,
                            self.sensor_base_rotation)

    def _publish_odom_msg(self, d):
        odom = self._make_odom(d)
        self.odom_pub.publish(odom)

        if self.frame_layout == 'odin':
            sensor_xyz, sensor_quat = self._sensor_pose(d)
            if self.odin_odom_pub is not None:
                self.odin_odom_pub.publish(
                    self._make_odom(d, self.imu_frame, (sensor_xyz, sensor_quat)))
            child = self.imu_frame
            t_xyz, t_quat = sensor_xyz, sensor_quat
        else:
            if self.odin_odom_pub is not None:
                self.odin_odom_pub.publish(odom)
            child = self.base_frame
            t_xyz = (odom.pose.pose.position.x,
                     odom.pose.pose.position.y,
                     odom.pose.pose.position.z)
            t_quat = (odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = child
        t.transform.translation.x = float(t_xyz[0])
        t.transform.translation.y = float(t_xyz[1])
        t.transform.translation.z = float(t_xyz[2])
        t.transform.rotation.x = float(t_quat[0])
        t.transform.rotation.y = float(t_quat[1])
        t.transform.rotation.z = float(t_quat[2])
        t.transform.rotation.w = float(t_quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, d):
        self.stats['pose'] += 1
        try:
            self._pose_hist.append((float(d['ts']), float(d['x']), float(d['y']),
                                    float(d['z']), float(d['qx']), float(d['qy']),
                                    float(d['qz']), float(d['qw'])))
        except (KeyError, TypeError, ValueError):
            pass
        if self.gt_pub is not None:
            self.gt_pub.publish(self._make_odom(d))
        if self.odom_impair is None:
            self._publish_odom_msg(d)
            return
        for item in self.odom_impair.process(d, time.monotonic()):
            self._publish_odom_msg(item)

    def _publish_cloud_msg(self, d, camera=True):
        xyz = d.get('xyz')
        if xyz is None:
            return
        arr = np.asarray(xyz, dtype=np.float32)
        if arr.size == 0:
            return
        if arr.ndim != 2:
            arr = arr.reshape(-1, 3)

        msg = PointCloud2()
        msg.header.stamp = self._stamp(d['ts'])
        msg.header.frame_id = d.get('frame_id', 'map')
        msg.height = 1
        msg.width = int(arr.shape[0])
        use_lut = self._lut_keys is not None and self._lut_voxel > 0
        use_camera = camera and self.camera_color
        if use_lut or use_camera:
            cols = np.full((len(arr), 3), 180, dtype=np.uint8)
            if use_lut:
                q = np.floor(arr / self._lut_voxel).astype(np.int64) + (1 << 20)
                keys = q[:, 0] | (q[:, 1] << 21) | (q[:, 2] << 42)
                pos = np.searchsorted(self._lut_keys, keys)
                pos = np.clip(pos, 0, len(self._lut_keys) - 1)
                hit = self._lut_keys[pos] == keys
                cols[hit] = self._lut_colors[pos[hit]]
            if use_camera:
                cols = self._apply_camera_colors(arr, d, cols)
            packed = ((cols[:, 0].astype(np.uint32) << 16)
                      | (cols[:, 1].astype(np.uint32) << 8)
                      | cols[:, 2].astype(np.uint32))
            # Single float32 rgb field, identical layout to the real Odin cloud.
            data = np.empty((len(arr), 4), dtype=np.float32)
            data[:, :3] = arr
            data[:, 3] = packed.view(np.float32)
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 16
        else:
            data = np.ascontiguousarray(arr)
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            point_step = 12
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * msg.width
        msg.is_dense = True
        # 直接写 numpy 字节，避免 tolist() 的开销（10Hz × 数万点）
        msg.data = np.ascontiguousarray(data).tobytes()
        self.pc_pub.publish(msg)
        if self.odin_cloud_pub is not None:
            self.odin_cloud_pub.publish(msg)

    def _publish_pointcloud(self, d):
        self.stats['cloud'] += 1
        if self.cloud_impair is None:
            self._publish_cloud_msg(d)
            return
        if self._odom_rigid_cloud and self.odom_impair is not None:
            dx, dy = self.odom_impair.current_drift()
            d = dict(d)
            if d.get('xyz') is not None:
                d['xyz'] = np.asarray(d['xyz'], dtype=np.float32).reshape(-1, 3) + np.array(
                    [dx, dy, 0.0], dtype=np.float32)
        for item in self.cloud_impair.process(d, time.monotonic()):
            self._publish_cloud_msg(item, camera=False)

    def _publish_image(self, d):
        if not self.publish_image:
            return
        msg = CompressedImage()
        msg.header.stamp = self._stamp(d['ts'])
        msg.header.frame_id = d.get('frame_id', 'camera')
        msg.format = 'jpeg'
        msg.data = bytes(d['data'])
        self.img_pub.publish(msg)
        if self.odin_img_pub is not None:
            self.odin_img_pub.publish(msg)
        self.stats['image'] += 1
        if self.camera_color:
            intr = d.get('intrinsics')
            b2c = d.get('body_to_camera')
            if intr and b2c and np.isfinite(float(d['ts'])):
                self._img_hist.append((float(d['ts']), bytes(d['data']), intr, b2c))

    def _decode_jpeg(self, stamp, jpeg):
        if self._decoded_image is not None and self._decoded_image[0] == stamp:
            return self._decoded_image[1]
        try:
            from PIL import Image
            image = np.asarray(Image.open(io.BytesIO(jpeg)).convert('RGB'))
        except Exception as exc:  # noqa: BLE001 - keep LUT colors on bad frames
            self.get_logger().warn(f'camera color: JPEG decode failed: {exc}',
                                   throttle_duration_sec=5.0)
            return None
        self._decoded_image = (stamp, image)
        return image

    def _apply_camera_colors(self, pts, d, cols):
        """Project the nearest image onto map-frame points (static-scene coloring)."""
        if not self._img_hist or not self._pose_hist:
            return cols
        try:
            ts = float(d['ts'])
        except (KeyError, TypeError, ValueError):
            return cols
        idx = nearest_stamp_index([h[0] for h in self._img_hist], ts)
        if idx is None:
            return cols
        img_ts, jpeg, intr, b2c = self._img_hist[idx]
        if abs(img_ts - ts) > self.camera_color_max_age:
            return cols
        pid = nearest_stamp_index([p[0] for p in self._pose_hist], img_ts)
        if pid is None:
            return cols
        p = self._pose_hist[pid]
        if abs(p[0] - img_ts) > 0.15:
            return cols
        image = self._decode_jpeg(img_ts, jpeg)
        if image is None:
            return cols
        quat = b2c['quat_xyzw']
        cam_xyz, cam_quat = compose_transform(
            [p[1], p[2], p[3]], [p[7], p[4], p[5], p[6]],
            b2c['xyz'], [quat[3], quat[0], quat[1], quat[2]])
        colors, mask = colorize_points(pts, image, cam_xyz, cam_quat, intr)
        if mask.any():
            cols = cols.copy()
            cols[mask] = colors[mask]
        return cols

    def _publish_imu(self, d):
        if self.imu_impair is not None:
            d = self.imu_impair.distort(d)
        msg = Imu()
        msg.header.stamp = self._stamp(d['ts'])
        msg.header.frame_id = d.get('frame_id', 'imu')
        msg.linear_acceleration.x = float(d.get('ax', 0.0))
        msg.linear_acceleration.y = float(d.get('ay', 0.0))
        msg.linear_acceleration.z = float(d.get('az', 0.0))
        msg.angular_velocity.x = float(d.get('wx', 0.0))
        msg.angular_velocity.y = float(d.get('wy', 0.0))
        msg.angular_velocity.z = float(d.get('wz', 0.0))
        if 'qx' in d:
            msg.orientation.x = float(d['qx'])
            msg.orientation.y = float(d['qy'])
            msg.orientation.z = float(d['qz'])
            msg.orientation.w = float(d['qw'])
        else:
            msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(msg)
        if self.odin_imu_pub is not None:
            self.odin_imu_pub.publish(msg)
        self.stats['imu'] += 1

    def close(self):
        self._stop.set()
        self._disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    th = threading.Thread(target=node.socket_loop, daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        th.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
