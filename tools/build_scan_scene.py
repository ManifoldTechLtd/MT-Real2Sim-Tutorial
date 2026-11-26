#!/usr/bin/env python3
"""Build an Isaac scene (USDZ) and a 2D navigation map from MindCloud/Q9000 exports.

Pipeline (per scene):
  mesh.ply (binary PLY, x/y/z/nx/ny/nz + triangles)
      -> USDZ mesh with vertex colors sampled from the PCD
  point cloud (PCD, binary or binary_compressed, fields include x/y/z[/rgb])
      -> PGM/YAML occupancy grid using a local-floor height band

The scene keeps the original world frame; the navigation stack only needs the
2D (x, y) frame to match the mesh. Set the Isaac environment position z to
`-floor_z` (reported in the metadata JSON) so the floor sits at z=0.

Usage:
  python3 build_scan_scene.py --name floor2_hall \
      --mesh /path/mesh.ply --pcd /path/cloud.pcd --out-dir /path/out
"""

import argparse
import json
import math
import os
import time

import lzf
import numpy as np
from scipy.ndimage import distance_transform_edt


DTYPES = {
    (4, 'F'): np.float32, (8, 'F'): np.float64,
    (4, 'U'): np.uint32, (4, 'I'): np.int32,
    (2, 'U'): np.uint16, (2, 'I'): np.int16,
    (1, 'U'): np.uint8, (1, 'I'): np.int8,
}


def read_pcd_header(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError('unexpected EOF before DATA')
            s = line.decode('ascii', 'ignore').strip()
            if not s or s.startswith('#'):
                continue
            if s.startswith('DATA'):
                header['DATA'] = s.split()[1].lower()
                return header, f.tell()
            parts = s.split(None, 1)
            if len(parts) == 2:
                header[parts[0]] = parts[1]


def pcd_layout(header):
    fields = header.get('FIELDS', 'x y z').split()
    sizes = [int(v) for v in header.get('SIZE', '4 4 4').split()]
    types = header.get('TYPE', 'F F F').split()
    counts = [int(v) for v in header.get('COUNT', ' '.join(['1'] * len(fields))).split()]
    num = int(header.get('POINTS', header.get('WIDTH', '0')))
    return fields, sizes, types, counts, num


def iter_pcd(path, chunk_points=2_000_000):
    """Yield (xyz float32 [N,3], rgb uint8 [N,3] or None) chunks."""
    header, offset = read_pcd_header(path)
    fields, sizes, types, counts, num = pcd_layout(header)
    fmt = header['DATA']
    rgb_idx = fields.index('rgb') if 'rgb' in fields else None

    if fmt == 'binary':
        dt_list = []
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            base = DTYPES.get((sz, tp.upper()), np.uint8)
            dt_list.append((f'f{len(dt_list)}', base, (cnt,)) if cnt > 1 else
                           (f'f{len(dt_list)}', base))
        dtype = np.dtype(dt_list)
        point_step = dtype.itemsize
        file_size = os.path.getsize(path)
        avail = (file_size - offset) // point_step
        num = min(num, avail)
        arr = np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(num,))
        for start in range(0, num, chunk_points):
            block = arr[start:start + chunk_points]
            xyz = np.stack([block[f'f{fields.index(a)}'] for a in ('x', 'y', 'z')],
                           axis=1).astype(np.float32)
            rgb = None
            if rgb_idx is not None:
                packed = np.asarray(block[f'f{rgb_idx}']).view(np.uint32)
                rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255,
                                packed & 255], axis=1).astype(np.uint8)
            yield xyz, rgb
        return

    if fmt == 'binary_compressed':
        with open(path, 'rb') as f:
            f.seek(offset)
            comp = int.from_bytes(f.read(4), 'little')
            uncomp = int.from_bytes(f.read(4), 'little')
            raw = lzf.decompress(f.read(comp), uncomp)
        cols = []
        cur = 0
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            nbytes = sz * cnt * num
            cols.append((name, sz, tp, cnt, cur, nbytes))
            cur += nbytes
        col = {name: (sz, tp, cnt, start, nbytes) for name, sz, tp, cnt, start, nbytes in cols}
        for start in range(0, num, chunk_points):
            n = min(chunk_points, num - start)
            xyz = np.empty((n, 3), dtype=np.float32)
            for j, axis in enumerate(('x', 'y', 'z')):
                sz, tp, cnt, base, _ = col[axis]
                dt = DTYPES.get((sz, tp.upper()), np.float32)
                off = base + start * sz * cnt
                xyz[:, j] = np.frombuffer(raw, dtype=dt, count=n, offset=off)
            rgb = None
            if rgb_idx is not None:
                sz, tp, cnt, base, _ = col['rgb']
                dt = DTYPES.get((sz, tp.upper()), np.uint32)
                off = base + start * sz * cnt
                packed = np.frombuffer(raw, dtype=dt, count=n, offset=off).view(np.uint32)
                rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255,
                                packed & 255], axis=1).astype(np.uint8)
            yield xyz, rgb
        return

    raise ValueError(f'unsupported PCD DATA={fmt}')


def read_ply(path):
    """Read a binary PLY mesh (vertex x/y/z[/nx/ny/nz] + triangular faces)."""
    with open(path, 'rb') as f:
        hdr = b''
        while b'end_header' not in hdr:
            hdr += f.read(4096)
        end = hdr.index(b'end_header') + len(b'end_header\n')
        head = hdr[:end].decode('ascii', 'ignore')
        lines = [l.strip() for l in head.splitlines()]
        fmt = lines[1].split()[1]
        if fmt not in ('binary_big_endian', 'binary_little_endian'):
            raise ValueError(f'unsupported PLY format {fmt}')
        bo = '>' if fmt == 'binary_big_endian' else '<'
        nv = int([l for l in lines if l.startswith('element vertex')][0].split()[-1])
        nf = int([l for l in lines if l.startswith('element face')][0].split()[-1])
        props = []
        in_vertex = False
        for l in lines:
            if l.startswith('element vertex'):
                in_vertex = True
                continue
            if l.startswith('element face'):
                break
            if in_vertex and l.startswith('property'):
                p = l.split()
                props.append((p[1], p[2]))
        vdt = np.dtype([(name, bo + 'f4') for _, name in props])
        f.seek(end)
        verts = np.frombuffer(f.read(nv * vdt.itemsize), dtype=vdt)
        verts = np.stack([verts['x'], verts['y'], verts['z']], axis=1).astype(np.float32)
        fdt = np.dtype([('n', 'u1'), ('a', bo + 'i4'), ('b', bo + 'i4'), ('c', bo + 'i4')])
        faces = np.frombuffer(f.read(nf * fdt.itemsize), dtype=fdt)
        if not np.all(faces['n'] == 3):
            raise ValueError('PLY contains non-triangle faces; convert to triangles first')
        idx = np.stack([faces['a'], faces['b'], faces['c']], axis=1).astype(np.int64)
    return verts, idx


def voxel_keys(xyz, voxel):
    q = np.floor(xyz / voxel).astype(np.int64)
    q += (1 << 20)
    return q[:, 0] | (q[:, 1] << 21) | (q[:, 2] << 42)


def build_color_lookup(points_xyz, points_rgb, voxel):
    keys = voxel_keys(points_xyz, voxel)
    order = np.argsort(keys, kind='stable')
    keys = keys[order]
    rgb = points_rgb[order].astype(np.float32)
    uniq, starts, inv = np.unique(keys, return_index=True, return_inverse=True)
    count = np.bincount(inv, minlength=len(uniq)).astype(np.float32)
    colors = np.stack([
        np.bincount(inv, weights=rgb[:, c], minlength=len(uniq)) / count
        for c in range(3)
    ], axis=1)
    return uniq, (colors / 255.0).astype(np.float32)


def sample_colors(path, stride, max_points):
    xs, rs = [], []
    total = 0
    for xyz, rgb in iter_pcd(path, chunk_points=2_000_000):
        if rgb is None:
            return None
        if stride > 1:
            xyz, rgb = xyz[::stride], rgb[::stride]
        xs.append(xyz)
        rs.append(rgb)
        total += len(xyz)
        if total >= max_points:
            break
    if not xs:
        return None
    return np.concatenate(xs), np.concatenate(rs)


def build_map(path, resolution, out_prefix, min_height=0.30, max_height=0.90,
              free_band=(-0.30, 3.0), sample_stride=17, fixed_floor=None):
    t0 = time.time()
    # pass 1: bounds from a sample (robust to scan outliers)
    sx, sy = [], []
    floor_samples = []
    for xyz, _ in iter_pcd(path, chunk_points=2_000_000):
        step = max(1, len(xyz) // 500_000)
        sel = xyz[::step]
        if fixed_floor is not None:
            rel = sel[:, 2] - fixed_floor
            sel = sel[(rel >= free_band[0]) & (rel <= free_band[1])]
            if len(sel) == 0:
                continue
        sx.append(sel[:, 0]); sy.append(sel[:, 1]); floor_samples.append(sel[:, 2])
    sx = np.concatenate(sx); sy = np.concatenate(sy); sz = np.concatenate(floor_samples)
    x0, x1 = np.percentile(sx, [0.1, 99.9])
    y0, y1 = np.percentile(sy, [0.1, 99.9])
    floor_z = float(fixed_floor) if fixed_floor is not None else float(np.percentile(sz, 2.0))
    pad = 1.0
    x_min, x_max = float(x0 - pad), float(x1 + pad)
    y_min, y_max = float(y0 - pad), float(y1 + pad)
    w = int(math.ceil((x_max - x_min) / resolution))
    h = int(math.ceil((y_max - y_min) / resolution))
    print(f'[map] bounds x[{x_min:.2f},{x_max:.2f}] y[{y_min:.2f},{y_max:.2f}] '
          f'{w}x{h} floor_z={floor_z:.3f}')

    free = np.zeros(w * h, dtype=bool)
    occ = np.zeros(w * h, dtype=bool)
    if fixed_floor is not None:
        for xyz, _ in iter_pcd(path, chunk_points=2_000_000):
            ix = np.floor((xyz[:, 0] - x_min) / resolution).astype(np.int64)
            iy = np.floor((xyz[:, 1] - y_min) / resolution).astype(np.int64)
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            if not np.any(ok):
                continue
            cell = iy[ok] * w + ix[ok]
            rel = xyz[ok, 2] - fixed_floor
            free[cell[(rel >= free_band[0]) & (rel <= free_band[1])]] = True
            occ[cell[(rel >= min_height) & (rel <= max_height)]] = True
    else:
        minz = np.full(w * h, np.inf, dtype=np.float32)
        for xyz, _ in iter_pcd(path, chunk_points=2_000_000):
            ix = np.floor((xyz[:, 0] - x_min) / resolution).astype(np.int64)
            iy = np.floor((xyz[:, 1] - y_min) / resolution).astype(np.int64)
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            if not np.any(ok):
                continue
            cell = iy[ok] * w + ix[ok]
            z = xyz[ok, 2]
            order = np.argsort(cell, kind='stable')
            cell_s = cell[order]; z_s = z[order]
            starts = np.flatnonzero(np.r_[True, cell_s[1:] != cell_s[:-1]])
            mins = np.minimum.reduceat(z_s, starts)
            uniq = cell_s[starts]
            cur = minz[uniq]
            minz[uniq] = np.minimum(cur, mins)
        for xyz, _ in iter_pcd(path, chunk_points=2_000_000):
            ix = np.floor((xyz[:, 0] - x_min) / resolution).astype(np.int64)
            iy = np.floor((xyz[:, 1] - y_min) / resolution).astype(np.int64)
            ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            if not np.any(ok):
                continue
            cell = iy[ok] * w + ix[ok]
            rel = xyz[ok, 2] - minz[cell]
            free[cell[(rel >= free_band[0]) & (rel <= free_band[1])]] = True
            occ[cell[(rel >= min_height) & (rel <= max_height)]] = True

    grid = np.full(w * h, 205, dtype=np.uint8)
    grid[free] = 254
    grid[occ] = 0
    grid = grid.reshape(h, w)

    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    pgm = out_prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(np.flipud(grid).tobytes())
    with open(out_prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{x_min:.6f}, {y_min:.6f}, 0.0]\n')
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    # spawn candidates: free cells with the largest clearance, near the map center
    clear = distance_transform_edt(grid != 0) * resolution
    clear[grid != 254] = 0
    cy, cx = h // 2, w // 2
    yy, xx = np.mgrid[0:h, 0:w]
    score = clear - 0.002 * np.hypot((xx - cx) * resolution, (yy - cy) * resolution)
    k = np.argmax(score)
    sy_, sx_ = np.unravel_index(k, (h, w))
    spawn = (x_min + (sx_ + 0.5) * resolution, y_min + (sy_ + 0.5) * resolution)
    stats = {
        'resolution': resolution, 'width': w, 'height': h,
        'origin': [x_min, y_min], 'floor_z': floor_z,
        'scene_position_z': -floor_z, 'spawn_xy': [spawn[0], spawn[1]],
        'occupied_cells': int((grid == 0).sum()), 'free_cells': int((grid == 254).sum()),
        'unknown_cells': int((grid == 205).sum()), 'elapsed_sec': round(time.time() - t0, 1),
    }
    print(f'[map] spawn candidate ({spawn[0]:.2f}, {spawn[1]:.2f}) '
          f'clear={clear[sy_, sx_]:.2f}m')
    return stats


def write_usdz(mesh_ply, color_pcd, out_usdz, color_voxel, decimate_voxel,
               color_stride, color_max_points):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdUtils, Vt

    verts, faces = read_ply(mesh_ply)
    print(f'[usdz] mesh vertices={len(verts)} faces={len(faces)}')
    colors = None
    if color_pcd:
        sampled = sample_colors(color_pcd, color_stride, color_max_points)
        if sampled is not None:
            pts, rgb = sampled
            uniq, lut = build_color_lookup(pts, rgb, color_voxel)
            keys = voxel_keys(verts, color_voxel)
            pos = np.searchsorted(uniq, keys)
            pos = np.clip(pos, 0, len(uniq) - 1)
            hit = uniq[pos] == keys
            colors = np.full((len(verts), 3), 0.55, dtype=np.float32)
            colors[hit] = lut[pos[hit]]
            colors = np.where(colors <= 0.04045, colors / 12.92,
                              ((colors + 0.055) / 1.055) ** 2.4)
            print(f'[usdz] vertex colors: {hit.mean() * 100:.1f}% matched')

    if decimate_voxel > 0:
        q = np.floor(verts / decimate_voxel).astype(np.int64) + (1 << 20)
        keys = q[:, 0] | (q[:, 1] << 21) | (q[:, 2] << 42)
        uniq, inv = np.unique(keys, return_inverse=True)
        cnt = np.bincount(inv).astype(np.float32)
        nv = np.stack([np.bincount(inv, weights=verts[:, c]) for c in range(3)], axis=1) / cnt[:, None]
        if colors is not None:
            colors = np.stack([np.bincount(inv, weights=colors[:, c]) for c in range(3)], axis=1) / cnt[:, None]
        faces = inv[faces]
        keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
        faces = faces[keep]
        verts = nv.astype(np.float32)
        print(f'[usdz] decimated vertices={len(verts)} faces={len(faces)}')

    usdc = out_usdz[:-5] + '.usdc' if out_usdz.endswith('.usdz') else out_usdz + '.usdc'
    stage = Usd.Stage.CreateNew(usdc)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, '/World/Mesh')
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(verts, dtype=np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(faces.reshape(-1), dtype=np.int32)))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    ext = np.stack([verts.min(axis=0), verts.max(axis=0)], axis=0)
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(ext.astype(np.float32)))
    if colors is not None:
        pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            'displayColor', Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(colors, dtype=np.float32)))
    stage.GetRootLayer().Save()
    UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(usdc), out_usdz)
    os.remove(usdc)
    print(f'[usdz] wrote {out_usdz} ({os.path.getsize(out_usdz)/1e6:.1f} MB)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--mesh', required=True)
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--color-voxel', type=float, default=0.08)
    ap.add_argument('--decimate-voxel', type=float, default=0.0)
    ap.add_argument('--color-stride', type=int, default=1)
    ap.add_argument('--color-max-points', type=int, default=8_000_000)
    ap.add_argument('--skip-usdz', action='store_true',
                    help='Only build the navigation map (USDZ already produced by export_usdz.py)')
    ap.add_argument('--fixed-floor', type=float, default=None,
                    help='Use this floor z for the map (multi-level scans); default local-floor')
    ap.add_argument('--min-height', type=float, default=0.30)
    ap.add_argument('--max-height', type=float, default=0.90)
    ap.add_argument('--free-min', type=float, default=-0.30)
    ap.add_argument('--free-max', type=float, default=3.00)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    usdz = os.path.join(args.out_dir, args.name + '.usdz')
    if not args.skip_usdz:
        write_usdz(args.mesh, args.pcd, usdz, args.color_voxel, args.decimate_voxel,
                   args.color_stride, args.color_max_points)
    stats = build_map(args.pcd, args.resolution,
                      os.path.join(args.out_dir, args.name + '_sim'),
                      min_height=args.min_height, max_height=args.max_height,
                      free_band=(args.free_min, args.free_max),
                      fixed_floor=args.fixed_floor)
    stats.update({'name': args.name, 'usdz': usdz, 'mesh': args.mesh, 'pcd': args.pcd})
    meta = os.path.join(args.out_dir, args.name + '_meta.json')
    with open(meta, 'w') as f:
        json.dump(stats, f, indent=2)
    print(json.dumps(stats, indent=2))
    print(f'[done] {meta}')


if __name__ == '__main__':
    main()
