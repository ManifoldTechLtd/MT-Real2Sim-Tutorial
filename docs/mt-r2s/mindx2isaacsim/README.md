# 3DGS + Mesh → Isaac Sim USDZ Conversion Tool

Convert **3DGS PLY** (Gaussian visual) and **Mesh PLY** (collision body) from the [MindCloudX AI](https://www.mindcloudx.com/) platform into a single USDZ file for import into NVIDIA Isaac Sim for physics simulation.

---

## 1. Overview

### Input

| File | Source | Coordinate System | Purpose |
|------|--------|-------------------|---------|
| `3dgs.ply` | MindCloudX cloud 3DGS training | **Z-up** (download Z-up from the platform) | Visual rendering (NuRec Gaussian) |
| `mesh.ply` | MindCloudX mesh reconstruction | **Z-up** | Collision detection (PhysX) |

### Output

| File | Description |
|------|-------------|
| `scene.usdz` | 3DGS visual + invisible collision mesh (import this into Isaac Sim) |

### Key Processing

- **Mesh face decimation**: `--simplify_faces` prevents PhysX collision failure on high-poly meshes

---

## 2. Environment Setup

Python **3.10~3.12** (`open3d` does not yet provide prebuilt packages for Python 3.13+).

If your system Python version is outside this range, create a conda environment:

```bash
conda create -n mindcloudx2usdz python=3.12 -y
conda activate mindcloudx2usdz
```

Install dependencies:

```bash
pip install -r requirements.txt
```

| Package | Purpose |
|---------|---------|
| `numpy` | Array computation |
| `plyfile` | PLY file parsing |
| `open3d` | Mesh face decimation |
| `usd-core` | OpenUSD Python bindings (USDZ generation) |
| `msgpack` | NuRec model data packing |
| `scipy` | SH coefficient coordinate transform |

> If Isaac Sim is already installed, you can use its built-in Python environment (which includes `pxr`), eliminating the need to install `usd-core` separately.

### File Descriptions

| File | Function |
|------|----------|
| `export_usdz.py` | Entry point — converts 3DGS PLY + Mesh PLY → USDZ |
| `ply_to_usdz.py` | Internal module: 3DGS PLY → NuRec USDZ |
| `add_mesh_to_usdz.py` | Internal module: inject mesh collision body into USDZ |

---

## 3. Usage

### 3.1 Prepare Input

Download both the **Z-up 3DGS PLY** and the **Mesh PLY** from the MindCloudX platform:

```
my_scene/
├── ply/
│   └── 3dgs.ply      ← 3DGS PLY (Z-up)
└── mesh/
    └── mesh.ply       ← Mesh PLY (Z-up)
```

### 3.2 Run Conversion

```bash
python export_usdz.py \
    --ply_dir ~/data/my_scene/ply \
    --mesh_dir ~/data/my_scene/mesh \
    -o ~/data/my_scene/usdz
```

Outputs `scene.usdz` (3DGS visual + collision mesh).

> Defaults: `--simplify_faces 500000`, `--max_dist 200`. Usually no manual specification needed.

### 3.3 Parameter Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--ply_dir` | (required) | 3DGS PLY directory (looks for `3dgs.ply` or first `*.ply`) |
| `--mesh_dir` | (required) | Mesh PLY directory (looks for `mesh.ply` or first `*.ply`) |
| `-o` | (required) | USDZ output directory |
| `--simplify_faces` | 500000 | Mesh decimation target face count (0 = no decimation) |
| `--max_dist` | 200 | Remove Gaussians farther than this from origin (0 = no filter) |
| `--max_sh_degree` | 3 | Maximum SH (spherical harmonics) degree |
| `--scale_offset` | 0 | Log-scale offset (e.g., `ln(scale_factor)`) |
| `--collision_approx` | `none` | Collision approximation mode, **must remain `none`** |
| `--mesh_visible` | false | Whether mesh is visible (default: invisible, collision only) |
| `--is_2dgs` | false | Force 2DGS mode (default: auto-detect) |
| `--export_filtered_ply` | false | Also export distance-filtered PLY |
| `--y_up` | false | Enable Y-up → Z-up transform (for legacy data only, see below) |

### 3.4 Import into Isaac Sim

1. Open Isaac Sim
2. `File → Add Reference`, select `scene.usdz`
3. The scene displays 3DGS visual + invisible collision mesh
4. In the Property panel, confirm `PhysicsCollisionAPI` is enabled

---

## 4. Important Notes

### ⚠️ Excessive Mesh Face Count Causes Pass-Through

When mesh exceeds approximately **5 million faces**, PhysX collision silently fails. Use `--simplify_faces 500000` to decimate to 500K faces. Recommended range: **500K ~ 1M faces**.

### ⚠️ collision_approx Must Be `none`

For meshes imported via USDZ into Isaac Sim, only `none` (exact triangle collision) works correctly. Keep the default value.

### ⚠️ Mesh Face Count Must Not Be Too Low

Collision mesh should have **≥ 50 faces/m²** (by bounding box surface area). Too few faces causes localized pass-through.

---

## 5. Quick Checklist

| # | Check Item |
|---|------------|
| 1 | Download **Z-up** 3DGS PLY and Mesh PLY from MindCloudX |
| 2 | Mesh PLY is Z-up, metric units, triangulated |
| 3 | Mesh decimation defaults to 500K faces (skip with `--simplify_faces 0`) |
| 4 | `collision_approx` remains `none` (default) |
| 5 | Run `export_usdz.py` to generate USDZ |
| 6 | Isaac Sim `Add Reference` → `scene.usdz` |
| 7 | Verify: place object above mesh, Play, confirm no pass-through |

---

## 6. Legacy: Y-up 3DGS Data

Early versions of the MindCloudX platform exported 3DGS PLY files in **Y-up** coordinate system. The platform now provides **Z-up** files directly — always download the Z-up version when available.

If you are working with legacy Y-up 3DGS data, enable the coordinate transform with `--y_up`:

```bash
python export_usdz.py \
    --ply_dir ~/data/my_scene/ply \
    --mesh_dir ~/data/my_scene/mesh \
    -o ~/data/my_scene/usdz \
    --y_up
```

This automatically converts positions `(x, y, z) → (x, z, -y)`, rotates quaternions, and applies Wigner-D rotation to SH coefficients.

---

## Maintainer

Ethan TU — [Manifoldtech](https://www.manifoldtech.cn/)
