"""
Standalone converter: gsplat PLY → NuRec USDZ for Isaac Sim / Omniverse.

Extracts Gaussian parameters from a standard 3DGS/2DGS PLY file and packages
them into a NuRec-compatible USDZ that can be loaded in Isaac Sim.

No 3DGUT installation required — only needs: plyfile, msgpack, numpy, usd-core.

Usage:
    conda activate o2s_gs
    python gsplat_ply_to_usdz.py input.ply -o output.usdz

Based on NVIDIA 3DGUT export pipeline (Apache 2.0 License).
"""

import gzip
import io
import logging
import os
import tempfile
import zipfile
from pathlib import Path

import msgpack
import numpy as np
from plyfile import PlyData
from pxr import Gf, Sdf, Usd, UsdGeom, UsdUtils, UsdVol

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 1. PLY Importer (adapted from 3DGUT PLYImporter)
# ---------------------------------------------------------------------------

def load_ply(path: str, max_sh_degree: int = 3, scale_offset: float = 0.0, max_dist: float = 0.0):
    """Load a standard 3DGS/2DGS PLY file and return Gaussian attributes.
    
    Args:
        max_dist: If > 0, remove Gaussians whose position is farther than
                  max_dist from the origin (useful for filtering r≈1000
                  spherical-shell placeholders from Q9000 cloud training).
    """
    logger.info(f"Loading PLY: {path}")
    plydata = PlyData.read(path)
    el = plydata.elements[0]
    n = el.count

    # Positions
    positions = np.stack([el["x"], el["y"], el["z"]], axis=1).astype(np.float32)

    # Remove NaN/Inf positions
    finite_mask = np.isfinite(positions).all(axis=1)
    if not finite_mask.all():
        n_bad = int((~finite_mask).sum())
        logger.info(f"Removing {n_bad} Gaussians with NaN/Inf positions")
        positions = positions[finite_mask]
        el_data = el.data if hasattr(el, 'data') else el
        el = el_data[finite_mask]
        n = len(positions)

    # Distance filter
    if max_dist > 0:
        dists = np.linalg.norm(positions, axis=1)
        mask = dists <= max_dist
        n_before = n
        n = int(mask.sum())
        logger.info(f"Distance filter (max_dist={max_dist}): {n_before} -> {n} Gaussians "
                    f"(removed {n_before - n})")
        # Build a filtered view for downstream attribute reads
        positions = positions[mask]
        el_arr = el.data if hasattr(el, 'count') else el
        _el_data = el_arr[mask]
    else:
        _el_data = el.data if hasattr(el, 'count') else el
    el = _el_data

    # Opacity (density)
    densities = _el_data["opacity"].astype(np.float32).reshape(-1, 1)

    # Albedo (SH DC coefficients)
    albedo = np.stack([_el_data["f_dc_0"], _el_data["f_dc_1"], _el_data["f_dc_2"]], axis=1).astype(np.float32)

    # Specular (higher-order SH)
    extra_f_names = sorted(
        [name for name in _el_data.dtype.names if name.startswith("f_rest_")],
        key=lambda x: int(x.split("_")[-1]),
    )
    num_speculars = (max_sh_degree + 1) ** 2 - 1  # 15 for degree 3

    if len(extra_f_names) > 0:
        actual_sh_degree = int(np.sqrt(len(extra_f_names) // 3 + 1)) - 1
    else:
        actual_sh_degree = 0

    specular = np.zeros((n, num_speculars * 3), dtype=np.float32)
    if len(extra_f_names) > 0:
        actual_speculars = len(extra_f_names) // 3
        temp = np.zeros((n, len(extra_f_names)), dtype=np.float32)
        for idx, name in enumerate(extra_f_names):
            temp[:, idx] = _el_data[name].astype(np.float32)
        # Channel-major → feature-major layout
        temp = temp.reshape(n, 3, actual_speculars).transpose(0, 2, 1).reshape(n, actual_speculars * 3)
        specular[:, : actual_speculars * 3] = temp

    # Scales
    scale_names = sorted(
        [name for name in _el_data.dtype.names if name.startswith("scale_")],
        key=lambda x: int(x.split("_")[-1]),
    )
    scales = np.stack([_el_data[s].astype(np.float32) for s in scale_names], axis=1)

    # Apply scale offset (e.g. log(scale_factor) when moving from normalized to metric space)
    if scale_offset != 0.0:
        logger.info(f"Applying scale offset: {scale_offset:.4f} (exp={np.exp(scale_offset):.4f}x)")
        scales += scale_offset

    # Rotations (quaternions wxyz)
    rot_names = sorted(
        [name for name in _el_data.dtype.names if name.startswith("rot_")],
        key=lambda x: int(x.split("_")[-1]),
    )
    rotations = np.stack([_el_data[r].astype(np.float32) for r in rot_names], axis=1)

    logger.info(f"Loaded {n} Gaussians, SH degree {actual_sh_degree}, "
                f"scales dim {scales.shape[1]}")

    return {
        "positions": positions,
        "rotations": rotations,
        "scales": scales,
        "densities": densities,
        "albedo": albedo,
        "specular": specular,
        "sh_degree": actual_sh_degree,
        "num_gaussians": n,
    }


# ---------------------------------------------------------------------------
# 1b. Y-up → Z-up coordinate transform
# ---------------------------------------------------------------------------

def _compute_sh_rotation_matrices(R, max_l=3):
    """Compute real-SH Wigner-D rotation matrices for l=0..max_l.

    Given a 3×3 rotation matrix R, returns a list of numpy arrays
    [D_0, D_1, ..., D_{max_l}] where D_l has shape (2l+1, 2l+1).

    D_l satisfies:  Y_l^{m'}(R @ d) = sum_m  D_l[m', m] * Y_l^m(d)
    """
    from scipy.special import sph_harm_y as _sph_harm_y

    def _real_sh(l, m, theta, phi):
        if m > 0:
            return float(np.real(_sph_harm_y(l, m, theta, phi) * np.sqrt(2) * (-1)**m))
        elif m < 0:
            return float(np.imag(_sph_harm_y(l, -m, theta, phi) * np.sqrt(2) * (-1)**(-m)))
        else:
            return float(np.real(_sph_harm_y(l, 0, theta, phi)))

    rng = np.random.RandomState(42)
    n_samples = 200
    dirs = rng.randn(n_samples, 3).astype(np.float64)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs_rot = (R @ dirs.T).T

    results = []
    for l in range(max_l + 1):
        dim = 2 * l + 1
        ms = list(range(-l, l + 1))
        Y_orig = np.zeros((n_samples, dim), dtype=np.float64)
        Y_rot = np.zeros((n_samples, dim), dtype=np.float64)
        for i in range(n_samples):
            th = np.arccos(np.clip(dirs[i, 2], -1, 1))
            ph = np.arctan2(dirs[i, 1], dirs[i, 0])
            th_r = np.arccos(np.clip(dirs_rot[i, 2], -1, 1))
            ph_r = np.arctan2(dirs_rot[i, 1], dirs_rot[i, 0])
            for j, m in enumerate(ms):
                Y_orig[i, j] = _real_sh(l, m, th, ph)
                Y_rot[i, j] = _real_sh(l, m, th_r, ph_r)
        D_T, _, _, _ = np.linalg.lstsq(Y_orig, Y_rot, rcond=None)
        D = D_T.T
        D[np.abs(D) < 1e-12] = 0.0
        results.append(D)
    return results


def apply_y_up_to_z_up(attrs, max_sh_degree=3):
    """Transform 3DGS attributes from Y-up to Z-up coordinate system in-place.

    Position:   (x, y, z) → (x, z, -y)      [R_x(-90°)]
    Rotation:   left-multiply q_R = (√2/2, -√2/2, 0, 0) wxyz
    Scales:     unchanged (local to each Gaussian)
    SH DC:      unchanged
    SH l≥1:     Wigner-D(R_x(+90°)) = D(R⁻¹) per band per color channel
    """
    n = attrs["num_gaussians"]

    # --- Positions ---
    pos = attrs["positions"]
    new_pos = np.empty_like(pos)
    new_pos[:, 0] = pos[:, 0]
    new_pos[:, 1] = pos[:, 2]
    new_pos[:, 2] = -pos[:, 1]
    attrs["positions"] = new_pos

    # --- Rotations ---
    sq = np.float32(np.sqrt(2.0) / 2.0)
    rw, rx, ry, rz = (attrs["rotations"][:, i].copy() for i in range(4))
    new_rots = np.empty_like(attrs["rotations"])
    new_rots[:, 0] = sq * rw + sq * rx
    new_rots[:, 1] = sq * rx - sq * rw
    new_rots[:, 2] = sq * ry + sq * rz
    new_rots[:, 3] = sq * rz - sq * ry
    attrs["rotations"] = new_rots

    # --- SH coefficients (specular) ---
    sh_degree = attrs["sh_degree"]
    if sh_degree == 0:
        return
    specular = attrs["specular"]
    num_speculars = (max_sh_degree + 1) ** 2 - 1
    actual_speculars = (sh_degree + 1) ** 2 - 1

    # specular is feature-major interleaved (N, K*3): [c0_R,c0_G,c0_B, c1_R,...]
    # Reshape to (N, K, 3) then transpose to channel-major (N, 3, K)
    spec_feat = specular[:, :actual_speculars * 3].reshape(n, actual_speculars, 3)
    spec_ch = spec_feat.transpose(0, 2, 1).copy()

    # SH rotates with D(R⁻¹): positions use R_x(-90°), so SH uses D(R_x(+90°))
    R_sh = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
    D_mats = _compute_sh_rotation_matrices(R_sh, max_l=sh_degree)

    offset = 0
    for l in range(1, sh_degree + 1):
        dim = 2 * l + 1
        D = D_mats[l].astype(np.float32)
        band = spec_ch[:, :, offset:offset + dim]
        spec_ch[:, :, offset:offset + dim] = np.einsum('ij,ncj->nci', D, band)
        offset += dim

    spec_feat_out = spec_ch.transpose(0, 2, 1).reshape(n, actual_speculars * 3)
    new_specular = np.zeros_like(specular)
    new_specular[:, :actual_speculars * 3] = spec_feat_out
    attrs["specular"] = new_specular.astype(specular.dtype)

    logger.info(f"Applied Y-up → Z-up transform ({n} Gaussians, SH degree {sh_degree})")


# ---------------------------------------------------------------------------
# 2. NuRec Template (adapted from 3DGUT templates.py)
# ---------------------------------------------------------------------------

def build_nurec_template(attrs: dict, is_2dgs: bool = False) -> dict:
    """Build the NuRec msgpack template from Gaussian attributes."""
    dtype = np.float16

    state_dict = {
        "._extra_state": {"obj_track_ids": {"gaussians": []}},
    }

    # Tensor data
    for key, field in [
        (".gaussians_nodes.gaussians.positions", "positions"),
        (".gaussians_nodes.gaussians.rotations", "rotations"),
        (".gaussians_nodes.gaussians.scales", "scales"),
        (".gaussians_nodes.gaussians.densities", "densities"),
        (".gaussians_nodes.gaussians.features_albedo", "albedo"),
        (".gaussians_nodes.gaussians.features_specular", "specular"),
    ]:
        arr = attrs[field]
        state_dict[key] = arr.astype(dtype).tobytes()
        state_dict[key + ".shape"] = list(arr.shape)

    # Extra signal (empty)
    extra = np.zeros((attrs["num_gaussians"], 0), dtype=dtype)
    state_dict[".gaussians_nodes.gaussians.extra_signal"] = extra.tobytes()
    state_dict[".gaussians_nodes.gaussians.extra_signal.shape"] = list(extra.shape)

    # n_active_features
    state_dict[".gaussians_nodes.gaussians.n_active_features"] = (
        np.array([attrs["sh_degree"]], dtype=np.int64).tobytes()
    )
    state_dict[".gaussians_nodes.gaussians.n_active_features.shape"] = []

    # 2DGS optimizations:
    # 1. density_kernel_planar=True makes the renderer treat them as 2D disks
    # 2. density_kernel_min_response reduces clipping at disk edges
    density_kernel_planar = True if is_2dgs else False
    min_response = 0.005 if is_2dgs else 0.0113

    template = {
        "nre_data": {
            "version": "0.2.576",
            "model": "nre",
            "config": {
                "layers": {
                    "gaussians": {
                        "name": "sh-gaussians",
                        "device": "cuda",
                        "density_activation": "sigmoid",
                        "scale_activation": "exp",
                        "rotation_activation": "normalize",
                        "precision": 16,
                        "particle": {
                            "density_kernel_planar": density_kernel_planar,
                            "density_kernel_degree": 2,
                            "density_kernel_density_clamping": True,
                            "density_kernel_min_response": min_response,
                            "radiance_sph_degree": 3,
                        },
                        "transmittance_threshold": 0.0001,
                    }
                },
                "renderer": {
                    "name": "3dgut-nrend",
                    "log_level": 3,
                    "force_update": False,
                    "update_step_train_batch_end": False,
                    "per_ray_features": False,
                    "global_z_order": True,
                    "projection": {
                        "n_rolling_shutter_iterations": 5,
                        "ut_dim": 3,
                        "ut_alpha": 1.0,
                        "ut_beta": 2.0,
                        "ut_kappa": 0.0,
                        "ut_require_all_sigma_points": False,
                        "image_margin_factor": 0.1,
                        "min_projected_ray_radius": 0.5477225575051661,
                    },
                    "culling": {
                        "rect_bounding": True,
                        "tight_opacity_bounding": True,
                        "tile_based": True,
                        "near_clip_distance": 1e-8,
                        "far_clip_distance": 3.402823466e38,
                    },
                    "render": {"mode": "kbuffer", "k_buffer_size": 0},
                },
                "name": "gaussians_primitive",
                "appearance_embedding": {
                    "name": "skip-appearance",
                    "embedding_dim": 0,
                    "device": "cuda",
                },
                "background": {
                    "name": "skip-background",
                    "device": "cuda",
                    "composite_in_linear_space": False,
                },
            },
            "state_dict": state_dict,
        }
    }
    return template


# ---------------------------------------------------------------------------
# 3. USD Stage creation (adapted from 3DGUT serializer.py)
# ---------------------------------------------------------------------------

def create_nurec_usd_stage(nurec_filename: str, positions: np.ndarray):
    """Create the USD stage with NuRec Volume prim."""
    min_coord = positions.min(axis=0).astype(float)
    max_coord = positions.max(axis=0).astype(float)
    logger.info(f"AABB: min={min_coord}, max={max_coord}")

    stage = Usd.Stage.CreateInMemory()
    stage.SetMetadata("metersPerUnit", 1.0)
    stage.SetMetadata("upAxis", "Z")
    stage.SetTimeCodesPerSecond(24.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetMetadata("defaultPrim", "World")

    # Render settings
    render_settings = {
        "rtx:rendermode": "RaytracedLighting",
        "rtx:directLighting:sampledLighting:samplesPerPixel": 8,
        "rtx:post:histogram:enabled": False,
        "rtx:post:registeredCompositing:invertToneMap": True,
        "rtx:post:registeredCompositing:invertColorCorrection": True,
        "rtx:material:enableRefraction": False,
        "rtx:post:tonemap:op": 2,
        "rtx:raytracing:fractionalCutoutOpacity": False,
        "rtx:matteObject:visibility:secondaryRays": True,
    }
    stage.SetMetadataByDictKey("customLayerData", "renderSettings", render_settings)

    # Volume prim
    gauss_path = "/World/gauss"
    gauss_volume = UsdVol.Volume.Define(stage, gauss_path)
    gauss_prim = gauss_volume.GetPrim()

    # Identity transform
    matrix_op = gauss_volume.AddTransformOp()
    matrix_op.Set(Gf.Matrix4d(1.0))

    # NuRec attributes
    gauss_prim.CreateAttribute("omni:nurec:isNuRecVolume", Sdf.ValueTypeNames.Bool).Set(True)
    gauss_prim.CreateAttribute("omni:nurec:useProxyTransform", Sdf.ValueTypeNames.Bool).Set(False)

    # Field assets
    nurec_rel_path = "./" + nurec_filename

    density_path = gauss_path + "/density_field"
    density_field = stage.DefinePrim(density_path, "OmniNuRecFieldAsset")
    gauss_volume.CreateFieldRelationship("density", density_path)
    density_field.CreateAttribute("filePath", Sdf.ValueTypeNames.Asset).Set(nurec_rel_path)
    density_field.CreateAttribute("fieldName", Sdf.ValueTypeNames.Token).Set("density")
    density_field.CreateAttribute("fieldDataType", Sdf.ValueTypeNames.Token).Set("float")
    density_field.CreateAttribute("fieldRole", Sdf.ValueTypeNames.Token).Set("density")

    emissive_path = gauss_path + "/emissive_color_field"
    emissive_field = stage.DefinePrim(emissive_path, "OmniNuRecFieldAsset")
    gauss_volume.CreateFieldRelationship("emissiveColor", emissive_path)
    emissive_field.CreateAttribute("filePath", Sdf.ValueTypeNames.Asset).Set(nurec_rel_path)
    emissive_field.CreateAttribute("fieldName", Sdf.ValueTypeNames.Token).Set("emissiveColor")
    emissive_field.CreateAttribute("fieldDataType", Sdf.ValueTypeNames.Token).Set("float3")
    emissive_field.CreateAttribute("fieldRole", Sdf.ValueTypeNames.Token).Set("emissiveColor")

    # Color correction matrix (identity)
    emissive_field.CreateAttribute("omni:nurec:ccmR", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1, 0, 0, 0))
    emissive_field.CreateAttribute("omni:nurec:ccmG", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0, 1, 0, 0))
    emissive_field.CreateAttribute("omni:nurec:ccmB", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0, 0, 1, 0))

    # Extent and crop
    min_list = min_coord.tolist()
    max_list = max_coord.tolist()
    gauss_prim.GetAttribute("extent").Set([min_list, max_list])
    gauss_prim.CreateAttribute("omni:nurec:offset", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3d(0, 0, 0))
    gauss_prim.CreateAttribute("omni:nurec:crop:minBounds", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3d(*min_list))
    gauss_prim.CreateAttribute("omni:nurec:crop:maxBounds", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3d(*max_list))
    gauss_prim.CreateRelationship("proxy")

    return stage


def create_default_usd_stage(gauss_filename: str, gauss_stage):
    """Create default.usda that references the gauss stage."""
    stage = Usd.Stage.CreateInMemory()
    stage.SetMetadata("metersPerUnit", 1.0)
    stage.SetMetadata("upAxis", "Z")
    stage.SetTimeCodesPerSecond(24.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetMetadata("defaultPrim", "World")

    delegate = UsdUtils.CoalescingDiagnosticDelegate()
    prim = stage.OverridePrim(f"/World/{Path(gauss_filename).stem}")
    prim.GetReferences().AddReference(gauss_filename)

    # Copy render settings
    gauss_layer = gauss_stage.GetRootLayer()
    if "renderSettings" in gauss_layer.customLayerData:
        stage.SetMetadataByDictKey("customLayerData", "renderSettings",
                                   gauss_layer.customLayerData["renderSettings"])

    return stage


def stage_to_bytes(stage) -> bytes:
    """Export a USD stage to bytes."""
    with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
        tmp = f.name
    stage.GetRootLayer().Export(tmp)
    with open(tmp, "rb") as f:
        data = f.read()
    os.unlink(tmp)
    return data


def write_usdz(output_path: Path, nurec_data: bytes, nurec_filename: str,
               gauss_stage, default_stage):
    """Package everything into a USDZ zip file."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_STORED) as zf:
        # default.usda first (USDZ spec requirement)
        zf.writestr("default.usda", stage_to_bytes(default_stage))
        # NuRec model data
        zf.writestr(nurec_filename, nurec_data)
        # Gauss USD
        zf.writestr("gauss.usda", stage_to_bytes(gauss_stage))

    logger.info(f"USDZ written: {output_path} ({output_path.stat().st_size / 1e6:.1f} MB)")


# ---------------------------------------------------------------------------
# 4. Main
# ---------------------------------------------------------------------------

def save_filtered_ply(src_ply_path: str, dst_ply_path: str, max_dist: float):
    """Read a 3DGS PLY, remove Gaussians with position norm > max_dist, save."""
    from plyfile import PlyElement
    ply = PlyData.read(src_ply_path)
    v = ply['vertex']
    means = np.column_stack([v['x'], v['y'], v['z']]).astype(np.float32)
    dist = np.linalg.norm(means, axis=1)
    mask = dist <= max_dist
    filtered = v.data[mask]
    new_el = PlyElement.describe(filtered, 'vertex')
    Path(dst_ply_path).parent.mkdir(parents=True, exist_ok=True)
    PlyData([new_el], text=False).write(dst_ply_path)
    logger.info(f"Saved filtered PLY: {len(filtered):,}/{len(v.data):,} Gaussians → {dst_ply_path}")


def convert_ply_to_usdz(input_ply: str, output_usdz: str, max_sh_degree: int = 3, is_2dgs: bool = False, scale_offset: float = 0.0, max_dist: float = 0.0, export_filtered_ply: str = None, y_up: bool = False):
    """Full pipeline: gsplat PLY → NuRec USDZ."""
    output_path = Path(output_usdz)
    stem = output_path.stem

    # Export filtered PLY if requested
    if export_filtered_ply and max_dist > 0:
        save_filtered_ply(input_ply, export_filtered_ply, max_dist)

    # Load PLY
    attrs = load_ply(input_ply, max_sh_degree=max_sh_degree, scale_offset=scale_offset, max_dist=max_dist)

    # Y-up → Z-up coordinate transform
    if y_up:
        apply_y_up_to_z_up(attrs, max_sh_degree=max_sh_degree)

    # Auto-detect 2DGS if not specified
    if not is_2dgs:
        scales = attrs["scales"]
        if scales.shape[1] == 3:
            s2_mean = scales[:, 2].mean()
            s01_mean = (scales[:, 0].mean() + scales[:, 1].mean()) / 2.0
            if s2_mean < s01_mean - 3.0: # Heuristic: scale_2 is significantly smaller
                logger.info(f"Auto-detected 2DGS (scale_2={s2_mean:.2f} << scale_01={s01_mean:.2f})")
                is_2dgs = True

    # Build NuRec template and compress
    logger.info(f"Building NuRec template (2DGS={is_2dgs})...")
    template = build_nurec_template(attrs, is_2dgs=is_2dgs)

    logger.info("Compressing with msgpack + gzip...")
    buf = io.BytesIO()
    with gzip.GzipFile(fileobj=buf, mode="wb", compresslevel=0) as f:
        f.write(msgpack.packb(template))
    nurec_data = buf.getvalue()
    nurec_filename = stem + ".nurec"

    # Create USD stages
    logger.info("Creating USD stages...")
    gauss_stage = create_nurec_usd_stage(nurec_filename, attrs["positions"])
    default_stage = create_default_usd_stage("gauss.usda", gauss_stage)

    # Write USDZ
    write_usdz(output_path, nurec_data, nurec_filename, gauss_stage, default_stage)
    logger.info("Done!")


