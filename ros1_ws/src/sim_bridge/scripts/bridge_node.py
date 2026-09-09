#!/usr/bin/env python3
"""
bridge_node.py
TCP 客户端 → ROS1 Topics 桥接节点

订阅: /cmd_vel (geometry_msgs/Twist) → TCP CMD_VEL
发布: /state_estimation (nav_msgs/Odometry)
      /registered_scan (sensor_msgs/PointCloud2)
      /camera/image/compressed (sensor_msgs/CompressedImage)
      /imu/data (sensor_msgs/Imu)
      TF 拓扑 (base_link 为中心, 传感器偏移用静态 TF 表达):
        map → odom (静态, identity)
        odom → base_link (动态, 来自 POSE 消息, 即机器人 body 位姿)
        base_link → base_footprint (静态, [0,0,-base_height])
        base_link → odin1_base_link (静态, sensor_base_offset_xyz, 传感器基座=lidar)
        odin1_base_link → lidar1 (静态 identity, alias)
        base_link → camera1  (静态, camera_offset_xyz)
        base_link → imu_link (静态, imu_offset_xyz)

      静态偏移按 robot_type 从 config/tf_<robot_type>.yaml 加载,
      由 bridge.launch 通过 rosparam load 注入. 默认值在下方 ctor 仅作 fallback.
"""

import sys
import os
import socket
import time
import struct
import threading
import numpy as np

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage, Imu
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros

# 导入协议模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from protocol import (
    encode_cmd_vel, encode_reset, decode_message,
    MSG_POSE, MSG_POINTCLOUD, MSG_IMAGE, MSG_IMU, HEADER_SIZE,
)


class BridgeNode:
    def __init__(self):
        rospy.init_node("sim_bridge", anonymous=False)

        # 参数
        self.host = rospy.get_param("~tcp_host", "localhost")
        self.port = rospy.get_param("~tcp_port", 9090)
        self.reconnect_interval = rospy.get_param("~reconnect_interval", 2.0)

        # Frame 命名
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.footprint_frame = rospy.get_param("~footprint_frame", "base_footprint")
        self.sensor_base_frame = rospy.get_param("~sensor_base_frame", "odin1_base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "lidar1")
        self.camera_frame = rospy.get_param("~camera_frame", "camera1")
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")

        # 静态偏移 (仿真 calibration/pinhole 默认值, 单位 米)
        # base_link ≡ spot body link, 足底高度参数化 (默认 0.237 = 实测 Spot 站立)
        self.base_height = rospy.get_param("~base_height", 0.237)
        self.sensor_base_offset = rospy.get_param("~sensor_base_offset_xyz", [0.0, 0.0, 0.8])
        self.camera_offset = rospy.get_param("~camera_offset_xyz", [0.3, 0.0, 0.6])
        self.imu_offset = rospy.get_param("~imu_offset_xyz", [0.0, 0.0, 0.5])

        # Publishers
        self.odom_pub = rospy.Publisher("/state_estimation", Odometry, queue_size=10)
        self.pc_pub = rospy.Publisher("/registered_scan", PointCloud2, queue_size=5)
        self.img_pub = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=5)
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_static_tfs()

        # Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/sim_reset", Empty, self.reset_callback)

        # TCP 连接状态
        self.sock = None
        self.recv_buffer = bytearray()
        self.connected = False

        # cmd_vel 锁
        self.cmd_lock = threading.Lock()
        self.latest_cmd = None

        # reset 锁
        self.reset_lock = threading.Lock()
        self.reset_requested = False

        rospy.loginfo(f"[sim_bridge] Target: {self.host}:{self.port}")
        rospy.loginfo(f"[sim_bridge] TF: {self.map_frame} → {self.odom_frame} → {self.base_frame}")
        rospy.loginfo(f"[sim_bridge] Static: {self.footprint_frame} (z=-{self.base_height}), "
                      f"{self.sensor_base_frame}={self.sensor_base_offset} (≡ {self.lidar_frame}), "
                      f"{self.camera_frame}={self.camera_offset}, "
                      f"{self.imu_frame}={self.imu_offset}")

    def _make_static_tf(self, parent, child, xyz, time):
        """辅助: 构造一个纯平移、零旋转的 TransformStamped"""
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.w = 1.0  # identity rotation
        return t

    def _publish_static_tfs(self):
        """
        启动时发布所有静态 TF (拓扑在文件头 docstring).
        """
        now = rospy.Time.now()
        tfs = [
            # map → odom (仿真无 SLAM 漂移)
            self._make_static_tf(self.map_frame, self.odom_frame, [0, 0, 0], now),
            # base_link → base_footprint (足底在地面)
            self._make_static_tf(self.base_frame, self.footprint_frame,
                                 [0, 0, -self.base_height], now),
            # base_link → odin1_base_link (传感器基座)
            self._make_static_tf(self.base_frame, self.sensor_base_frame,
                                 self.sensor_base_offset, now),
            # odin1_base_link → lidar1 (alias, 与 lidar 物理坐标重合)
            self._make_static_tf(self.sensor_base_frame, self.lidar_frame,
                                 [0, 0, 0], now),
            # base_link → camera1
            self._make_static_tf(self.base_frame, self.camera_frame,
                                 self.camera_offset, now),
            # base_link → imu_link
            self._make_static_tf(self.base_frame, self.imu_frame,
                                 self.imu_offset, now),
        ]
        self.static_tf_broadcaster.sendTransform(tfs)

    def cmd_vel_callback(self, msg):
        """ROS /cmd_vel → 缓存, 由发送线程转发到 TCP"""
        with self.cmd_lock:
            self.latest_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        rospy.loginfo_throttle(2, f"[sim_bridge] /cmd_vel received: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} wz={msg.angular.z:.2f}")

    def reset_callback(self, msg):
        """ROS /sim_reset → 触发 Isaac Sim 复位到起点"""
        with self.reset_lock:
            self.reset_requested = True
        rospy.loginfo("[sim_bridge] Reset requested (to_start=True)")

    def connect(self):
        """尝试连接 TCP 服务端"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connected = True
            self.recv_buffer.clear()
            rospy.loginfo(f"[sim_bridge] Connected to {self.host}:{self.port}")
            return True
        except (socket.error, OSError) as e:
            self.sock = None
            self.connected = False
            return False

    def disconnect(self):
        """断开连接"""
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.connected = False
        self.recv_buffer.clear()

    def run(self):
        """主循环"""
        rate = rospy.Rate(200)  # 200Hz 轮询

        while not rospy.is_shutdown():
            if not self.connected:
                if not self.connect():
                    rospy.logwarn_throttle(5, "[sim_bridge] Waiting for Isaac Sim TCP server...")
                    rospy.sleep(self.reconnect_interval)
                    continue

            # 接收数据
            try:
                data = self.sock.recv(65536)
                if not data:
                    rospy.logwarn("[sim_bridge] Server disconnected")
                    self.disconnect()
                    continue
                self.recv_buffer.extend(data)
            except BlockingIOError:
                pass  # 非阻塞, 无数据
            except (ConnectionResetError, BrokenPipeError, OSError):
                rospy.logwarn("[sim_bridge] Connection lost")
                self.disconnect()
                continue

            # 解析消息
            while True:
                msg_type, payload, consumed = decode_message(bytes(self.recv_buffer))
                if msg_type is None:
                    break
                self.recv_buffer = self.recv_buffer[consumed:]
                self.handle_message(msg_type, payload)

            # 发送 cmd_vel
            with self.cmd_lock:
                cmd = self.latest_cmd
                self.latest_cmd = None
            if cmd is not None and self.connected:
                try:
                    # 临时切阻塞模式, 防止非阻塞 sendall 抛 BlockingIOError
                    self.sock.setblocking(True)
                    self.sock.sendall(encode_cmd_vel(*cmd))
                    self.sock.setblocking(False)
                except BlockingIOError:
                    # send buffer 暂满, 跳过本次 (不断连)
                    try:
                        self.sock.setblocking(False)
                    except Exception:
                        pass
                    rospy.logwarn_throttle(5, "[sim_bridge] cmd_vel send buffer full, skipping")
                except (ConnectionResetError, BrokenPipeError, OSError):
                    try:
                        self.sock.setblocking(False)
                    except Exception:
                        pass
                    self.disconnect()

            # 发送 reset
            with self.reset_lock:
                reset_req = self.reset_requested
                self.reset_requested = False
            if reset_req and self.connected:
                try:
                    self.sock.setblocking(True)
                    self.sock.sendall(encode_reset(to_start=True))
                    self.sock.setblocking(False)
                except (ConnectionResetError, BrokenPipeError, OSError):
                    try:
                        self.sock.setblocking(False)
                    except Exception:
                        pass
                    self.disconnect()

            rate.sleep()

    def handle_message(self, msg_type, payload):
        """处理收到的 TCP 消息"""
        if msg_type == MSG_POSE:
            self.publish_odometry(payload)
        elif msg_type == MSG_POINTCLOUD:
            self.publish_pointcloud(payload)
        elif msg_type == MSG_IMAGE:
            self.publish_image(payload)
        elif msg_type == MSG_IMU:
            self.publish_imu(payload)

    def publish_odometry(self, data):
        """发布 Odometry + TF (odom → base_link)"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(data["ts"])
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = data["x"]
        odom.pose.pose.position.y = data["y"]
        odom.pose.pose.position.z = data["z"]
        odom.pose.pose.orientation.x = data["qx"]
        odom.pose.pose.orientation.y = data["qy"]
        odom.pose.pose.orientation.z = data["qz"]
        odom.pose.pose.orientation.w = data["qw"]

        odom.twist.twist.linear.x = data.get("vx", 0.0)
        odom.twist.twist.linear.y = data.get("vy", 0.0)
        odom.twist.twist.linear.z = data.get("vz", 0.0)
        odom.twist.twist.angular.x = data.get("wx", 0.0)
        odom.twist.twist.angular.y = data.get("wy", 0.0)
        odom.twist.twist.angular.z = data.get("wz", 0.0)

        self.odom_pub.publish(odom)

        # TF: odom → odin1_base_link
        self.tf_broadcaster.sendTransform(
            (data["x"], data["y"], data["z"]),
            (data["qx"], data["qy"], data["qz"], data["qw"]),
            odom.header.stamp,
            self.base_frame,
            self.odom_frame,
        )

    def publish_pointcloud(self, data):
        """发布 PointCloud2"""
        xyz = data.get("xyz")
        if xyz is None or len(xyz) == 0:
            return

        header = rospy.Header()
        header.stamp = rospy.Time.from_sec(data["ts"])
        header.frame_id = data.get("frame_id", "map")

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        msg = pc2.create_cloud(header, fields, xyz.tolist())
        self.pc_pub.publish(msg)


    def publish_image(self, data):
        """发布 CompressedImage"""
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.from_sec(data["ts"])
        msg.header.frame_id = data.get("frame_id", "camera")
        msg.format = "jpeg"
        msg.data = data["data"]
        self.img_pub.publish(msg)

    def publish_imu(self, data):
        """发布 Imu"""
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(data["ts"])
        msg.header.frame_id = data.get("frame_id", "imu")

        msg.linear_acceleration.x = data.get("ax", 0.0)
        msg.linear_acceleration.y = data.get("ay", 0.0)
        msg.linear_acceleration.z = data.get("az", 0.0)

        msg.angular_velocity.x = data.get("wx", 0.0)
        msg.angular_velocity.y = data.get("wy", 0.0)
        msg.angular_velocity.z = data.get("wz", 0.0)

        if "qx" in data:
            msg.orientation.x = data["qx"]
            msg.orientation.y = data["qy"]
            msg.orientation.z = data["qz"]
            msg.orientation.w = data["qw"]
        else:
            # 无姿态数据, 设置协方差为 -1 表示无效
            msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(msg)


if __name__ == "__main__":
    try:
        node = BridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
