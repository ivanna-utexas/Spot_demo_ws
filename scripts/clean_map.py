#!/usr/bin/env python3
"""
clean_map.py — Voxel-consensus cleaning and outlier removal for 3D maps.

Pipeline:
  1. Load aligned maps
  2. Voxel-consensus: keep only voxels observed in >= min_passes
  3. Statistical outlier removal
  4. Height filtering to suppress transient objects
  5. Save cleaned map

Input:  maps/liorf/aligned/*.pcd (or .ply)
Output: maps/liorf/cleaned/<name>_cleaned.pcd

Usage:
  python3 scripts/clean_map.py --input maps/liorf/aligned/ --output maps/liorf/cleaned/
  python3 scripts/clean_map.py --input maps/liorf/aligned/ --output maps/liorf/cleaned/ --min-passes 2
"""
import argparse
import os
import sys
import glob

try:
    import open3d as o3d
    import numpy as np
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def find_map_files(input_dir):
    files = []
    for ext in ['*.pcd', '*.ply']:
        files.extend(glob.glob(os.path.join(input_dir, ext)))
    return sorted(files)


def voxel_consensus(clouds, voxel_size, min_passes):
    """Keep only voxels observed in at least min_passes clouds."""
    voxel_counts = {}

    for cloud in clouds:
        # Discretize to voxel grid
        points = np.asarray(cloud.points)
        voxels = np.floor(points / voxel_size).astype(int)
        unique_voxels = set(map(tuple, voxels))
        for v in unique_voxels:
            voxel_counts[v] = voxel_counts.get(v, 0) + 1

    # Keep voxels with sufficient observations
    keep_voxels = {v for v, c in voxel_counts.items() if c >= min_passes}

    # Merge all clouds and filter
    all_points = np.vstack([np.asarray(c.points) for c in clouds])
    all_colors = None
    if all(c.has_colors() for c in clouds):
        all_colors = np.vstack([np.asarray(c.colors) for c in clouds])

    voxel_indices = np.floor(all_points / voxel_size).astype(int)
    mask = np.array([tuple(v) in keep_voxels for v in voxel_indices])

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(all_points[mask])
    if all_colors is not None:
        merged.colors = o3d.utility.Vector3dVector(all_colors[mask])

    return merged


def clean_pipeline(input_dir, output_dir, voxel_size=0.1, min_passes=2,
                   height_min=-2.0, height_max=10.0,
                   nb_neighbors=20, std_ratio=2.0):
    """Full cleaning pipeline."""
    files = find_map_files(input_dir)
    if not files:
        print(f'No map files in {input_dir}')
        return

    print(f'Loading {len(files)} aligned maps...')
    clouds = []
    for f in files:
        cloud = o3d.io.read_point_cloud(f)
        print(f'  {os.path.basename(f)}: {len(cloud.points)} points')
        clouds.append(cloud)

    # Step 1: Voxel consensus (if multiple passes)
    if len(clouds) > 1 and min_passes > 1:
        print(f'\nVoxel consensus (voxel={voxel_size}m, min_passes={min_passes})...')
        merged = voxel_consensus(clouds, voxel_size, min_passes)
        print(f'  After consensus: {len(merged.points)} points')
    else:
        # Single map or no consensus needed
        all_points = np.vstack([np.asarray(c.points) for c in clouds])
        merged = o3d.geometry.PointCloud()
        merged.points = o3d.utility.Vector3dVector(all_points)
        print(f'\nMerged: {len(merged.points)} points')

    # Step 2: Height filtering
    print(f'Height filter: [{height_min}, {height_max}]m...')
    points = np.asarray(merged.points)
    mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
    merged = merged.select_by_index(np.where(mask)[0])
    print(f'  After height filter: {len(merged.points)} points')

    # Step 3: Statistical outlier removal
    print(f'Statistical outlier removal (k={nb_neighbors}, std={std_ratio})...')
    merged, inlier_idx = merged.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    print(f'  After outlier removal: {len(merged.points)} points')

    # Step 4: Voxel downsample
    print(f'Final voxel downsample ({voxel_size}m)...')
    merged = merged.voxel_down_sample(voxel_size)
    print(f'  Final: {len(merged.points)} points')

    # Save
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, 'cleaned_map.pcd')
    o3d.io.write_point_cloud(out_path, merged)
    print(f'\nSaved cleaned map: {out_path}')

    return out_path


def main():
    parser = argparse.ArgumentParser(description='3D map cleaning pipeline')
    parser.add_argument('--input', required=True, help='Dir with aligned maps')
    parser.add_argument('--output', required=True, help='Dir for cleaned output')
    parser.add_argument('--voxel-size', type=float, default=0.1,
                        help='Voxel size [m] for consensus and downsample')
    parser.add_argument('--min-passes', type=int, default=2,
                        help='Min observation passes for voxel consensus')
    parser.add_argument('--height-min', type=float, default=-2.0,
                        help='Min height [m] to keep')
    parser.add_argument('--height-max', type=float, default=10.0,
                        help='Max height [m] to keep')
    parser.add_argument('--nb-neighbors', type=int, default=20,
                        help='Statistical outlier removal: neighbor count')
    parser.add_argument('--std-ratio', type=float, default=2.0,
                        help='Statistical outlier removal: std ratio')
    args = parser.parse_args()

    if not HAS_OPEN3D:
        print('Open3D required. Install: pip3 install open3d')
        sys.exit(1)

    clean_pipeline(
        args.input, args.output,
        voxel_size=args.voxel_size,
        min_passes=args.min_passes,
        height_min=args.height_min,
        height_max=args.height_max,
        nb_neighbors=args.nb_neighbors,
        std_ratio=args.std_ratio,
    )


if __name__ == '__main__':
    main()
