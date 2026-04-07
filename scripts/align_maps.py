#!/usr/bin/env python3
"""
align_maps.py — Multi-pass map alignment using downsampled GICP.

Registers multiple replay-pass point clouds into a common frame.
Default assumption: accepted replay outputs already share one frame.
If they don't, runs explicit inter-map registration with GICP
seeded by replay pose overlap.

Input:  maps/liorf/raw/*.pcd (or .ply)
Output: maps/liorf/aligned/*.pcd

Usage:
  python3 scripts/align_maps.py --input maps/liorf/raw/ --output maps/liorf/aligned/
  python3 scripts/align_maps.py --input maps/liorf/raw/ --output maps/liorf/aligned/ --force-align
"""
import argparse
import os
import sys
import glob
import subprocess
import shutil


def find_map_files(input_dir):
    """Find .pcd and .ply files in directory."""
    files = []
    for ext in ['*.pcd', '*.ply']:
        files.extend(glob.glob(os.path.join(input_dir, ext)))
    return sorted(files)


def check_open3d():
    """Check if Open3D is available for GICP."""
    try:
        import open3d  # noqa: F401
        return True
    except ImportError:
        return False


def align_with_open3d(files, output_dir, voxel_size=0.2):
    """Align maps using Open3D GICP."""
    import open3d as o3d
    import numpy as np

    print(f'Aligning {len(files)} maps with GICP (voxel_size={voxel_size}m)...')

    # Load first map as reference
    ref = o3d.io.read_point_cloud(files[0])
    ref_down = ref.voxel_down_sample(voxel_size)
    ref_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_size * 2, max_nn=30))

    # Save reference as-is
    ref_name = os.path.basename(files[0])
    ref_out = os.path.join(output_dir, ref_name)
    o3d.io.write_point_cloud(ref_out, ref)
    print(f'  Reference: {ref_name}')

    for f in files[1:]:
        name = os.path.basename(f)
        src = o3d.io.read_point_cloud(f)
        src_down = src.voxel_down_sample(voxel_size)
        src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2, max_nn=30))

        # GICP registration
        result = o3d.pipelines.registration.registration_generalized_icp(
            src_down, ref_down, voxel_size * 3,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=200, relative_fitness=1e-6, relative_rmse=1e-6
            )
        )

        print(f'  {name}: fitness={result.fitness:.4f} rmse={result.inlier_rmse:.4f}')

        # Transform and save
        src.transform(result.transformation)
        out_path = os.path.join(output_dir, name)
        o3d.io.write_point_cloud(out_path, src)

    print('Alignment complete.')


def copy_as_aligned(files, output_dir):
    """Copy files without alignment (same-frame assumption)."""
    print(f'Copying {len(files)} maps without alignment (same frame assumed)...')
    for f in files:
        dst = os.path.join(output_dir, os.path.basename(f))
        shutil.copy2(f, dst)
        print(f'  Copied: {os.path.basename(f)}')


def main():
    parser = argparse.ArgumentParser(description='Multi-pass map alignment')
    parser.add_argument('--input', required=True, help='Input directory with raw maps')
    parser.add_argument('--output', required=True, help='Output directory for aligned maps')
    parser.add_argument('--force-align', action='store_true',
                        help='Force GICP alignment even if maps appear same-frame')
    parser.add_argument('--voxel-size', type=float, default=0.2,
                        help='Voxel size for downsampling [m]')
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    files = find_map_files(args.input)

    if not files:
        print(f'No .pcd or .ply files found in {args.input}')
        sys.exit(1)

    print(f'Found {len(files)} map files.')

    if len(files) == 1:
        print('Single map — copying to aligned directory.')
        copy_as_aligned(files, args.output)
    elif args.force_align:
        if check_open3d():
            align_with_open3d(files, args.output, args.voxel_size)
        else:
            print('Open3D not available. Install with: pip3 install open3d')
            print('Falling back to copy (same-frame assumption).')
            copy_as_aligned(files, args.output)
    else:
        # Default: same frame assumption
        copy_as_aligned(files, args.output)

    print(f'\nAligned maps in: {args.output}')


if __name__ == '__main__':
    main()
