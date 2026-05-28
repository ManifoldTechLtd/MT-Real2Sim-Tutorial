#!/usr/bin/env python3
"""
Unified USDZ export script.

Handles 3 cases based on available inputs:
  1. PLY + mesh  → scene.usdz (3DGS visual + collision mesh)
  2. PLY only    → scene.usdz (NuRec Gaussian visual)
  3. mesh only   → scene.usdz (collision mesh for physics sim)

Wraps ply_to_usdz.py and add_mesh_to_usdz.py into a single entry point.

Usage:
    python export_usdz.py --ply_dir ./ply --mesh_dir ./mesh -o ./usdz
    python export_usdz.py --ply_dir ./ply -o ./usdz
    python export_usdz.py --mesh_dir ./mesh -o ./usdz
"""

import argparse
import sys
import tempfile
from pathlib import Path

# Import from sibling modules
from ply_to_usdz import convert_ply_to_usdz
from add_mesh_to_usdz import add_mesh_to_usdz, mesh_only_to_usdz


def find_ply(directory: Path, hints=("3dgs.ply", "3dgs_y_up.ply")) -> Path | None:
    """Find a PLY file in directory, preferring hint names in order."""
    for h in hints:
        p = directory / h
        if p.exists():
            return p
    plys = sorted(directory.glob("*.ply"))
    return plys[0] if plys else None


def main():
    parser = argparse.ArgumentParser(
        description="Unified USDZ export: 3DGS/2DGS PLY and/or mesh PLY → USDZ",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Cases:
  --ply_dir + --mesh_dir  → scene.usdz (3DGS visual + collision mesh)
  --ply_dir only          → scene.usdz (visual only)
  --mesh_dir only         → scene.usdz (collision only)

Examples:
  python export_usdz.py --ply_dir ~/data/ply --mesh_dir ~/data/mesh -o ~/data/usdz
  python export_usdz.py --mesh_dir ~/data/mesh -o ~/data/usdz
""",
    )
    parser.add_argument("--ply_dir", type=str, default=None,
                        help="Directory containing 3DGS/2DGS PLY (looks for 3dgs.ply or first *.ply)")
    parser.add_argument("--mesh_dir", type=str, default=None,
                        help="Directory containing mesh PLY (looks for mesh.ply or first *.ply)")
    parser.add_argument("-o", "--output_dir", type=str, required=True,
                        help="Output directory for USDZ files")
    parser.add_argument("--max_dist", type=float, default=200.0,
                        help="Remove Gaussians farther than this from origin (default: 200, 0 = no filter)")
    parser.add_argument("--max_sh_degree", type=int, default=3,
                        help="Max SH degree for PLY loading (default: 3)")
    parser.add_argument("--is_2dgs", action="store_true",
                        help="Force 2DGS rendering mode in NuRec")
    parser.add_argument("--scale_offset", type=float, default=0.0,
                        help="Log-scale offset for PLY (e.g. ln(scale_factor))")
    parser.add_argument("--export_filtered_ply", action="store_true",
                        help="Also save distance-filtered PLY as 3dgs_metric.ply (requires --max_dist > 0)")
    parser.add_argument("--mesh_visible", action="store_true",
                        help="Keep mesh visible (default: invisible when combined with visual USDZ)")
    parser.add_argument("--collision_approx", type=str, default="none",
                        choices=["none", "convexDecomposition", "meshSimplification", "convexHull"],
                        help="PhysX collision approximation (default: none = exact triangle mesh, works with Isaac Sim USDZ)")
    parser.add_argument("--simplify_faces", type=int, default=500000,
                        help="Simplify collision mesh to this many faces via quadric decimation (default: 500000, 0 = no simplification)")
    parser.add_argument("--y_up", action=argparse.BooleanOptionalAction, default=True,
                        help="Input 3DGS PLY is Y-up; convert to Z-up (default: True, use --no-y_up to disable)")

    args = parser.parse_args()

    if not args.ply_dir and not args.mesh_dir:
        parser.error("At least one of --ply_dir or --mesh_dir is required")

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Resolve input files
    ply_file = None
    mesh_file = None

    if args.ply_dir:
        ply_dir = Path(args.ply_dir)
        if not ply_dir.is_dir():
            print(f"Error: PLY directory not found: {ply_dir}")
            sys.exit(1)
        ply_file = find_ply(ply_dir, hints=("3dgs.ply",))
        if not ply_file:
            print(f"Error: No PLY file found in {ply_dir}")
            sys.exit(1)
        print(f"[PLY]  {ply_file} ({ply_file.stat().st_size / 1e6:.1f} MB)")

    if args.mesh_dir:
        mesh_dir = Path(args.mesh_dir)
        if not mesh_dir.is_dir():
            print(f"Error: Mesh directory not found: {mesh_dir}")
            sys.exit(1)
        mesh_file = find_ply(mesh_dir, hints=("mesh.ply",))
        if not mesh_file:
            print(f"Error: No PLY file found in {mesh_dir}")
            sys.exit(1)
        print(f"[Mesh] {mesh_file} ({mesh_file.stat().st_size / 1e6:.1f} MB)")

    output_usdz = out_dir / "scene.usdz"

    # --- Case 1: PLY + mesh → scene.usdz (visual + collision) ---
    if ply_file and mesh_file:
        filtered_ply = str(out_dir.parent / "ply" / "3dgs_metric.ply") if args.export_filtered_ply and args.max_dist > 0 else None

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_visual = str(Path(tmp_dir) / "scene.usdz")

            print(f"\n=== Step 1/2: PLY → visual USDZ ===")
            convert_ply_to_usdz(
                str(ply_file), tmp_visual,
                max_sh_degree=args.max_sh_degree,
                is_2dgs=args.is_2dgs,
                scale_offset=args.scale_offset,
                max_dist=args.max_dist,
                export_filtered_ply=filtered_ply,
                y_up=args.y_up,
            )

            print(f"\n=== Step 2/2: Inject mesh → scene.usdz ===")
            add_mesh_to_usdz(
                tmp_visual, str(output_usdz),
                mesh_ply_path=str(mesh_file),
                set_collision=True,
                set_invisible=not args.mesh_visible,
                collision_approx=args.collision_approx,
                simplify_faces=args.simplify_faces,
            )

        print(f"\n=== Output ===")
        print(f"  scene.usdz : {output_usdz.stat().st_size / 1e6:.1f} MB")
        if filtered_ply and Path(filtered_ply).exists():
            print(f"  3dgs_metric.ply : {Path(filtered_ply).stat().st_size / 1e6:.1f} MB")

    # --- Case 2: PLY only → scene.usdz ---
    elif ply_file:
        filtered_ply = str(out_dir.parent / "ply" / "3dgs_metric.ply") if args.export_filtered_ply and args.max_dist > 0 else None

        print(f"\n=== PLY → scene.usdz ===")
        convert_ply_to_usdz(
            str(ply_file), str(output_usdz),
            max_sh_degree=args.max_sh_degree,
            is_2dgs=args.is_2dgs,
            scale_offset=args.scale_offset,
            max_dist=args.max_dist,
            export_filtered_ply=filtered_ply,
            y_up=args.y_up,
        )

        print(f"\n=== Output ===")
        print(f"  scene.usdz : {output_usdz.stat().st_size / 1e6:.1f} MB")
        if filtered_ply and Path(filtered_ply).exists():
            print(f"  3dgs_metric.ply : {Path(filtered_ply).stat().st_size / 1e6:.1f} MB")

    # --- Case 3: mesh only → scene.usdz ---
    elif mesh_file:
        print(f"\n=== Mesh → scene.usdz ===")
        mesh_only_to_usdz(
            str(mesh_file), str(output_usdz),
            set_collision=True,
            set_invisible=False,
            collision_approx=args.collision_approx,
            simplify_faces=args.simplify_faces,
        )

        print(f"\n=== Output ===")
        print(f"  scene.usdz : {output_usdz.stat().st_size / 1e6:.1f} MB")

    print("\nDone!")


if __name__ == "__main__":
    main()
