#!/usr/bin/env python3
"""
bridge_node.py
TCP 客户端 → ROS1 Topics 桥接节点

订阅: /cmd_vel (geometry_msgs/Twist) → TCP CMD_VEL
发布: /state_estimation (nav_msgs/Odometry)
      /registered_scan (sensor_msgs/PointCloud2)
      /camera/image/compressed (sensor_msgs/CompressedImage)
      /imu/data (sensor_msgs/Imu)
      TF: map → sensor
"""

import sys
import os
import socket
import time
import struct
import threading
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage, Imu
from std_msgs.msg import Empty
import sensor_msgs.point_cloud2 as pc2
import tf

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

        # Publishers
        self.odom_pub = rospy.Publisher("/state_estimation", Odometry, queue_size=10)
        self.pc_pub = rospy.Publisher("/registered_scan", PointCloud2, queue_size=5)
        self.img_pub = rospy.Publisher("/camera/image/compressed", CompressedImage, queue_size=5)
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/reset_robot", Empty, self.reset_callback)

        # TCP 连接状态
        self.sock = None
        self.recv_buffer = bytearray()
        self.connected = False
        self.last_data_time = 0.0       # timestamp of last received data
        self.data_timeout = 10.0        # seconds with no data before reconnecting

        # cmd_vel 锁
        self.cmd_lock = threading.Lock()
        self.latest_cmd = None

        rospy.loginfo(f"[sim_bridge] Target: {self.host}:{self.port}")

    def cmd_vel_callback(self, msg):
        """ROS /cmd_vel → 缓存, 由发送线程转发到 TCP"""
        with self.cmd_lock:
            self.latest_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        rospy.loginfo_throttle(2, f"[sim_bridge] /cmd_vel received: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} wz={msg.angular.z:.2f}")

    def reset_callback(self, msg):
        """ROS /reset_robot → 发送 RESET 消息到 TCP"""
        if self.connected and self.sock:
            try:
                self.sock.setblocking(True)
                self.sock.sendall(encode_reset())
                self.sock.setblocking(False)
                rospy.loginfo("[sim_bridge] RESET sent to Isaac Sim")
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                rospy.logwarn(f"[sim_bridge] Failed to send RESET: {e}")
                try:
                    self.sock.setblocking(False)
                except Exception:
                    pass

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
            self.last_data_time = time.time()
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
                self.last_data_time = time.time()
            except BlockingIOError:
                # No data available — check for data timeout
                if time.time() - self.last_data_time > self.data_timeout:
                    rospy.logwarn(f"[sim_bridge] No data for {self.data_timeout}s, reconnecting...")
                    self.disconnect()
                    continue
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
        """发布 Odometry + TF"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(data["ts"])
        odom.header.frame_id = "map"
        odom.child_frame_id = "sensor"

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

        # TF: map → sensor
        self.tf_broadcaster.sendTransform(
            (data["x"], data["y"], data["z"]),
            (data["qx"], data["qy"], data["qz"], data["qw"]),
            odom.header.stamp,
            "sensor",
            "map",
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
