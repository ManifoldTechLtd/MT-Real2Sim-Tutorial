#!/usr/bin/env python3
"""Build a voxel color lookup table (x/y/z -> RGB) from a colored PCD.

The simulation LiDAR returns XYZ only; the bridge uses this LUT to publish a
colored PointCloud2 (rgb field) like the real Odin driver.

Usage:
  python3 build_color_lut.py --pcd cloud.pcd --out lut.npz --voxel 0.1
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_scan_scene import iter_pcd, voxel_keys  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--voxel', type=float, default=0.1)
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--max-points', type=int, default=16_000_000)
    ap.add_argument('--translation', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'Z'), help='Source PCD to simulation world translation')
    args = ap.parse_args()
    if args.voxel <= 0 or not np.isfinite(args.translation).all():
        ap.error('voxel must be positive and translation must be finite')

    xs, cs = [], []
    total = 0
    for xyz, rgb in iter_pcd(args.pcd):
        if rgb is None:
            raise SystemExit('PCD has no rgb field')
        if args.stride > 1:
            xyz, rgb = xyz[::args.stride], rgb[::args.stride]
        xs.append(xyz)
        cs.append(rgb)
        total += len(xyz)
        if total >= args.max_points:
            break
    pts = np.concatenate(xs) + np.asarray(args.translation, dtype=np.float32)
    cols = np.concatenate(cs).astype(np.float64)
    keys = voxel_keys(pts, args.voxel)
    uniq, inv = np.unique(keys, return_inverse=True)
    cnt = np.bincount(inv).astype(np.float64)
    avg = np.stack([np.bincount(inv, weights=cols[:, c]) for c in range(3)],
                   axis=1) / cnt[:, None]
    colors = np.clip(avg, 0, 255).astype(np.uint8)
    if colors.max() == 0:
        raise SystemExit('PCD rgb is all zero; no usable color for LUT')
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    np.savez_compressed(args.out, keys=uniq.astype(np.int64),
                        colors=colors, voxel=np.float64(args.voxel))
    print(f'LUT: {len(uniq)} voxels from {len(pts)} points -> {args.out} '
          f'({os.path.getsize(args.out)/1e6:.1f} MB)')


if __name__ == '__main__':
    main()
