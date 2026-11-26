"""Pinhole camera projection for cloud coloring (ROS-independent, NumPy only).

The simulation renders through a USD camera: +X forward, +Y left, +Z up.
Pixels grow to the right and downwards, so u = cx - fx*Y/X, v = cy - fy*Z/X.
"""
import numpy as np


def quat_wxyz_to_matrix(q):
    w, x, y, z = np.asarray(q, dtype=np.float64)
    n = w*w + x*x + y*y + z*z
    if n <= 0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y + z*z), s*(x*y - w*z), s*(x*z + w*y)],
        [s*(x*y + w*z), 1 - s*(x*x + z*z), s*(y*z - w*x)],
        [s*(x*z - w*y), s*(y*z + w*x), 1 - s*(x*x + y*y)],
    ])


def quat_wxyz_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def compose_transform(xyz1, quat1_wxyz, xyz2, quat2_wxyz):
    """T1 * T2 for (translation, wxyz quaternion) rigid transforms."""
    r1 = quat_wxyz_to_matrix(quat1_wxyz)
    q2 = np.asarray(quat2_wxyz, dtype=np.float64)
    return (np.asarray(xyz1, dtype=np.float64) + r1 @ np.asarray(xyz2, dtype=np.float64),
            quat_wxyz_multiply(np.asarray(quat1_wxyz, dtype=np.float64), q2))


def nearest_stamp_index(stamps, target):
    if not len(stamps):
        return None
    return int(np.argmin(np.abs(np.asarray(stamps, dtype=np.float64) - float(target))))


def project_points(points, cam_xyz, cam_quat_wxyz, intrinsics, min_depth=0.15):
    """Project map-frame points through T_map_camera."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    rot = quat_wxyz_to_matrix(cam_quat_wxyz)
    cam = (pts - np.asarray(cam_xyz, dtype=np.float64)) @ rot
    x, y, z = cam[:, 0], cam[:, 1], cam[:, 2]
    fx = float(intrinsics['fx'])
    fy = float(intrinsics['fy'])
    width = intrinsics.get('width')
    height = intrinsics.get('height')
    cx = float(intrinsics.get('cx', 0.5 * width))
    cy = float(intrinsics.get('cy', 0.5 * height))
    with np.errstate(divide='ignore', invalid='ignore'):
        u = cx - fx * y / x
        v = cy - fy * z / x
    mask = (x > min_depth) & np.isfinite(u) & np.isfinite(v)
    if width is not None:
        mask &= (u >= 0) & (u < float(width))
    if height is not None:
        mask &= (v >= 0) & (v < float(height))
    return u, v, x, mask


def colorize_points(points, image, cam_xyz, cam_quat_wxyz, intrinsics,
                    min_depth=0.15):
    """Sample image RGB for projected points; nearest point wins per pixel."""
    u, v, depth, mask = project_points(
        points, cam_xyz, cam_quat_wxyz, intrinsics, min_depth=min_depth)
    colors = np.zeros((len(mask), 3), dtype=np.uint8)
    if image is None or not mask.any():
        return colors, mask

    width = int(image.shape[1])
    idx = np.flatnonzero(mask)
    pix = v[idx].astype(np.int64) * width + u[idx].astype(np.int64)
    order = idx[np.argsort(depth[idx], kind='stable')]
    pix_sorted = v[order].astype(np.int64) * width + u[order].astype(np.int64)
    _, first = np.unique(pix_sorted, return_index=True)
    keep = np.zeros(len(mask), dtype=bool)
    keep[order[first]] = True
    mask = keep

    rows = v[mask].astype(np.int64)
    cols = u[mask].astype(np.int64)
    colors[mask] = np.asarray(image, dtype=np.uint8)[rows, cols, :3]
    return colors, mask
