#!/usr/bin/env python3
"""Project a cleaned 3D map into a Nav2-compatible 2D occupancy grid."""
import argparse
import os
import sys
import math
import struct

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


def load_cloud(path):
    """Load point cloud from PCD or PLY."""
    return o3d.io.read_point_cloud(path)


def project_to_grid(cloud, resolution, height_min, height_max,
                    occupied_thresh, free_thresh):
    """Project 3D cloud to 2D occupancy grid."""
    points = np.asarray(cloud.points)

    # Height slice
    mask = (points[:, 2] >= height_min) & (points[:, 2] <= height_max)
    slice_pts = points[mask]

    if len(slice_pts) == 0:
        print('WARNING: no points in height slice!')
        return None, None, None

    # Grid bounds
    x_min, y_min = slice_pts[:, 0].min(), slice_pts[:, 1].min()
    x_max, y_max = slice_pts[:, 0].max(), slice_pts[:, 1].max()

    # Add margin
    margin = 2.0
    x_min -= margin
    y_min -= margin
    x_max += margin
    y_max += margin

    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    print(f'Grid: {width}x{height} cells ({resolution}m/cell)')
    print(f'Bounds: x=[{x_min:.1f}, {x_max:.1f}] y=[{y_min:.1f}, {y_max:.1f}]')

    # Rasterize: count points per cell
    grid_counts = np.zeros((height, width), dtype=int)
    for pt in slice_pts:
        gx = int((pt[0] - x_min) / resolution)
        gy = int((pt[1] - y_min) / resolution)
        gx = min(gx, width - 1)
        gy = min(gy, height - 1)
        grid_counts[gy, gx] += 1

    # Convert to occupancy: unknown=205, free=254, occupied=0
    # Cells with points above threshold = occupied
    # Cells near occupied cells but no points = free
    # Everything else = unknown
    point_threshold = 2  # minimum points to mark as occupied
    occupancy = np.full((height, width), 205, dtype=np.uint8)  # unknown

    # Mark occupied cells
    occupied_mask = grid_counts >= point_threshold
    occupancy[occupied_mask] = 0  # occupied

    # Mark free cells (neighborhood of occupied, but not occupied themselves)
    from scipy.ndimage import binary_dilation
    dilated = binary_dilation(occupied_mask, iterations=3)
    free_mask = dilated & ~occupied_mask
    occupancy[free_mask] = 254  # free

    # PGM is stored with row 0 at top, but map convention has y increasing up.
    # Flip vertically for PGM.
    occupancy_pgm = np.flipud(occupancy)

    origin = [x_min, y_min, 0.0]
    return occupancy_pgm, origin, (width, height)


def save_pgm(occupancy, path):
    """Save as PGM (P5 binary)."""
    h, w = occupancy.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(occupancy.tobytes())
    print(f'Saved PGM: {path} ({w}x{h})')


def save_yaml(pgm_filename, resolution, origin, path,
              free_thresh=0.25, occupied_thresh=0.65):
    """Save map metadata YAML."""
    with open(path, 'w') as f:
        f.write(f'image: {pgm_filename}\n')
        f.write(f'resolution: {resolution}\n')
        f.write(f'origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n')
        f.write(f'negate: 0\n')
        f.write(f'occupied_thresh: {occupied_thresh}\n')
        f.write(f'free_thresh: {free_thresh}\n')
    print(f'Saved YAML: {path}')


def main():
    parser = argparse.ArgumentParser(description='3D-to-2D map projection')
    parser.add_argument('--input', required=True, help='Cleaned 3D map (PCD/PLY)')
    parser.add_argument('--output', required=True, help='Output directory')
    parser.add_argument('--name', default='map', help='Output filename base')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Grid resolution [m/cell]')
    parser.add_argument('--height-min', type=float, default=-0.1,
                        help='Min height for 2D slice [m]')
    parser.add_argument('--height-max', type=float, default=0.5,
                        help='Max height for 2D slice [m]')
    parser.add_argument('--occupied-thresh', type=float, default=0.65)
    parser.add_argument('--free-thresh', type=float, default=0.25)
    parser.add_argument('--promote', action='store_true',
                        help='Copy final output to maps/final/')
    args = parser.parse_args()

    if not HAS_OPEN3D or not HAS_NUMPY:
        print('Requires: pip3 install open3d numpy scipy')
        sys.exit(1)

    os.makedirs(args.output, exist_ok=True)

    print(f'Loading: {args.input}')
    cloud = load_cloud(args.input)
    print(f'Points: {len(cloud.points)}')

    occupancy, origin, (w, h) = project_to_grid(
        cloud, args.resolution,
        args.height_min, args.height_max,
        args.occupied_thresh, args.free_thresh
    )

    if occupancy is None:
        print('Failed to generate grid.')
        sys.exit(1)

    pgm_name = f'{args.name}.pgm'
    yaml_name = f'{args.name}.yaml'

    save_pgm(occupancy, os.path.join(args.output, pgm_name))
    save_yaml(pgm_name, args.resolution, origin,
              os.path.join(args.output, yaml_name),
              args.free_thresh, args.occupied_thresh)

    # Promote to maps/final/
    if args.promote:
        ws_root = os.path.expanduser('~/dance_ws_pedestrian_tracking')
        final_dir = os.path.join(ws_root, 'maps', 'final')
        os.makedirs(final_dir, exist_ok=True)

        import shutil
        shutil.copy2(os.path.join(args.output, pgm_name),
                     os.path.join(final_dir, pgm_name))
        # Update yaml to reference the local pgm name
        save_yaml(pgm_name, args.resolution, origin,
                  os.path.join(final_dir, yaml_name),
                  args.free_thresh, args.occupied_thresh)
        print(f'\nPromoted to: {final_dir}/')

    print('\nDone.')


if __name__ == '__main__':
    main()
