#!/usr/bin/env python3
"""Publish packed-color test clouds for viewer diagnostics.

Usage (inside the nav container):
  python3 /integrations/ons2/tools/publish_color_test.py --ros-args -p use_sim_time:=true

Topics (frame base_link, four colored rows in front of the robot):
  /color_test_rgb      x,y,z + rgb  UINT32   -> Foxglove "BGR (packed)"
  /color_test_rgb_f32  x,y,z + rgb  FLOAT32  -> same mode, PCL-style float bits
  /color_test_rgba     x,y,z + rgba UINT32   -> Foxglove "BGRA (packed)"
  /color_test_markers  MarkerArray RGBA      -> control: colors must render
Enable all of them and try both modes: whichever topic shows colors tells you
which field type your viewer accepts.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

ROWS = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]], np.uint32)


def build_cloud(frame_id, stamp, mode):
    xs, ys, zs, cs = [], [], [], []
    for row, color in enumerate(ROWS):
        packed_color = int(color[0]) << 16 | int(color[1]) << 8 | int(color[2])
        for col in range(10):
            xs.append(1.2 + 0.15 * col)
            ys.append(-0.45 + 0.3 * row)
            zs.append(0.6)
            cs.append(packed_color)
    packed = np.asarray(cs, np.uint32)
    xyz = np.stack([xs, ys, zs], axis=1).astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height, msg.width = 1, len(xs)
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(xs)
    msg.is_dense = True
    if mode == 'rgba':
        data = np.empty((len(xs), 4), np.uint32)
        data[:, :3] = xyz.view(np.uint32)
        data[:, 3] = packed | np.uint32(0xFF000000)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.fields.append(PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1))
    elif mode == 'rgb_u32':
        data = np.empty((len(xs), 4), np.uint32)
        data[:, :3] = xyz.view(np.uint32)
        data[:, 3] = packed
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    else:
        data = np.empty((len(xs), 4), np.float32)
        data[:, :3] = xyz
        data[:, 3] = packed.view(np.float32)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.data = data.tobytes()
    return msg


class ColorTest(Node):
    def __init__(self):
        super().__init__('color_test_publisher')
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.pubs = {
            'rgb_u32': self.create_publisher(PointCloud2, '/color_test_rgb', 1),
            'rgb_f32': self.create_publisher(PointCloud2, '/color_test_rgb_f32', 1),
            'rgba': self.create_publisher(PointCloud2, '/color_test_rgba', 1),
        }
        self.marker_pub = self.create_publisher(MarkerArray, '/color_test_markers', 1)
        self.create_timer(0.5, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for mode, pub in self.pubs.items():
            pub.publish(build_cloud(self.frame_id, stamp, mode))
        markers = MarkerArray()
        for i, color in enumerate(ROWS):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = 'color_test'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = 1.5
            marker.pose.position.y = -0.45 + 0.3 * i
            marker.pose.position.z = 0.6
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color.r = float(color[0]) / 255.0
            marker.color.g = float(color[1]) / 255.0
            marker.color.b = float(color[2]) / 255.0
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = ColorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
