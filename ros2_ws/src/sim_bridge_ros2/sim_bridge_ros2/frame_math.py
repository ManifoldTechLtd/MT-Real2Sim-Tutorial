"""Pure pose math shared by sim_bridge (ROS1) and sim_bridge_ros2."""

import math


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return [vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx]


def normalize_quat(quat):
    norm = math.sqrt(sum(q * q for q in quat))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [q / norm for q in quat]


def invert_pose(xyz, quat):
    q = normalize_quat(quat)
    q_inv = [-q[0], -q[1], -q[2], q[3]]
    t_inv = _rotate(q_inv, [-c for c in xyz])
    return t_inv, q_inv


def compose_pose(p_body, q_body, t_sensor, q_sensor):
    qb = normalize_quat(q_body)
    ts = _rotate(qb, t_sensor)
    xyz = [p_body[i] + ts[i] for i in range(3)]
    quat = _quat_multiply(qb, normalize_quat(q_sensor))
    return xyz, quat
