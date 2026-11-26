import struct
import subprocess
from pathlib import Path
import sys

import lzf
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from build_scan_scene import iter_pcd


@pytest.mark.parametrize('storage', ['binary', 'binary_compressed'])
@pytest.mark.parametrize('color_type', ['F', 'U'])
def test_pcd_packed_rgb_survives_decoding_and_chunk_boundaries(tmp_path, storage, color_type):
    count = 64
    bits = np.tile(np.array([0x123456, 0xFFA020, 0, 0x00FF80], dtype='<u4'), 16)
    colors = bits.view('<f4') if color_type == 'F' else bits
    xyz = np.arange(count * 3, dtype='<f4').reshape(count, 3)
    header = (f'VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\n'
              f'TYPE F F F {color_type}\nCOUNT 1 1 1 1\nWIDTH {count}\n'
              f'HEIGHT 1\nPOINTS {count}\nDATA {storage}\n').encode()
    if storage == 'binary':
        points = np.empty(count, dtype=[('xyz', '<f4', (3,)), ('rgb', '<u4')])
        points['xyz'] = xyz
        points['rgb'] = bits
        payload = points.tobytes()
    else:
        payload = b''.join(xyz[:, axis].tobytes() for axis in range(3)) + colors.tobytes()
        compressed = lzf.compress(payload)
        assert compressed is not None
        payload = struct.pack('<II', len(compressed), len(payload)) + compressed
    path = tmp_path / 'colored.pcd'
    path.write_bytes(header + payload)

    chunks = list(iter_pcd(str(path), chunk_points=7))
    actual_xyz = np.concatenate([part[0] for part in chunks])
    actual_rgb = np.concatenate([part[1] for part in chunks])
    expected_rgb = np.tile(np.array([[18, 52, 86], [255, 160, 32],
                                     [0, 0, 0], [0, 255, 128]], dtype=np.uint8), (16, 1))
    np.testing.assert_array_equal(actual_xyz, xyz)
    np.testing.assert_array_equal(actual_rgb, expected_rgb)


def test_color_lut_applies_scene_translation_before_voxel_lookup(tmp_path):
    header = b'VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n'
    path = tmp_path / 'one_color.pcd'
    path.write_bytes(header + struct.pack('<fffI', 0.0, 0.0, 0.0, 0x123456))
    output = tmp_path / 'lut.npz'
    tool = Path(__file__).resolve().parents[1] / 'tools' / 'build_color_lut.py'
    subprocess.run([sys.executable, str(tool), '--pcd', str(path), '--out', str(output),
                    '--voxel', '0.5', '--translation', '1.0', '0.5', '0.0'], check=True)
    data = np.load(output)
    np.testing.assert_array_equal(data['colors'], [[18, 52, 86]])
    np.testing.assert_array_equal(data['keys'], [4611688217453789186])
