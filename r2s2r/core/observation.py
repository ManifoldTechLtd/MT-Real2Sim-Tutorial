"""Validation and body filtering for synchronized RTX lidar observations."""
import math

import numpy as np


def rigid_transform(translation, quaternion):
    """Return a rigid transform from translation and a normalized wxyz quaternion."""
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(t)):
        raise ValueError("invalid sensor extrinsics")
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError("zero sensor quaternion")
    w, x, y, z = q / norm
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = t
    return out


def filter_body_returns(points, lidar_to_world, lidar_to_body, size, margin=0.01,
                        body_shape="box", cylinder_radius=0.25, cylinder_offset=0.18):
    """Remove body returns using the scan pose, without clearing rays behind them."""
    body_to_world = lidar_to_world @ np.linalg.inv(lidar_to_body)
    local = (points - body_to_world[:3, 3]) @ body_to_world[:3, :3]
    if body_shape == "double_cylinder":
        dx = np.abs(local[:, 0]) - cylinder_offset
        inside = ((dx * dx + local[:, 1] ** 2 <= (cylinder_radius + margin) ** 2)
                  & (np.abs(local[:, 2]) <= size[2] * 0.5 + margin))
    elif body_shape == "box":
        inside = np.all(np.abs(local) <= np.asarray(size) * 0.5 + margin, axis=1)
    else:
        raise ValueError("unsupported body shape")
    return points[~inside]


def fresh_scan(observation, last_scan_ns, now, not_before, timeout=1.0):
    stamp = observation.get('measurement_stamp', float('nan'))
    return (math.isfinite(stamp) and not_before < stamp <= now
            and now-stamp <= timeout and observation.get('scan_stamp_ns', -1) > last_scan_ns)


def decode_lidar_frame(frame):
    """Decode one render-product writer batch; do not infer times from polling.

    The selected solid-state profile fires together. Frames spanning more than
    1 ms are rejected because the local map uses a single ray origin.
    """
    scan = frame.get('IsaacCreateRTXLidarScanBuffer', {})
    points = np.asarray(scan.get('data', []), dtype=np.float32)
    if points.size == 0:
        return {}
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('invalid lidar point array')
    timestamps = np.asarray(scan.get('timestamp', []))
    distances = np.asarray(scan.get('distance', []))
    # Isaac 5.1's timestamp buffer need not contain one entry per XYZ return.
    # This simultaneous-fire profile needs only scan identity and time span;
    # never invent a per-point mapping by repeating or truncating timestamps.
    if (timestamps.ndim != 1 or timestamps.size == 0 or timestamps.dtype != np.dtype('uint64')
            or distances.shape != (len(points),)):
        raise ValueError('lidar requires uint64 scan timestamps and point-aligned distances')
    valid = np.all(np.isfinite(points), axis=1) & np.isfinite(distances) & (distances > 0)
    if not np.any(valid):
        return {}
    if np.any(timestamps == 0):
        raise ValueError('lidar scan contains zero timestamps')
    first, last = int(timestamps.min()), int(timestamps.max())
    if last - first > 1_000_000:
        raise ValueError('lidar frame spans more than 1 ms; per-ray origins required')
    pose = np.asarray(scan.get('info', {}).get('transform', []), dtype=float)
    if pose.size != 16:
        raise ValueError('missing scan measurement pose')
    pose = pose.reshape(4,4).T.copy()
    if (not np.all(np.isfinite(pose)) or not np.allclose(pose[3], [0,0,0,1], atol=1e-6)
            or not np.allclose(pose[:3,:3].T @ pose[:3,:3], np.eye(3), atol=1e-4)
            or np.linalg.det(pose[:3,:3]) < 0):
        raise ValueError('invalid scan measurement pose')
    stamp = float(frame['IsaacReadSimulationTime']['simulationTime'])
    if not math.isfinite(stamp) or stamp < 0:
        raise ValueError('invalid lidar measurement time')
    return {'point_cloud': points[valid].copy(), 'lidar_to_world': pose,
            'scan_stamp_ns': last, 'measurement_stamp': stamp}
