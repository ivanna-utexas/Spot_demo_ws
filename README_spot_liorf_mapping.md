# Spot LiORF Mapping — Qualification & Bringup

## Overview

This document tracks the qualification of [LiORF](https://github.com/YJZLuckyBoy/liorf) (LIO-SAM with Ring and FALS features) as a 3D lidar-inertial SLAM backend for Spot + VLP-16 on ROS 2 Humble, running inside Docker on a Jetson Orin.

**Status:** Phase 0 — First Evidence

## Architecture

```
Velodyne VLP-16 (192.168.50.201)
       │
       ▼
  /velodyne_points (PointCloud2)
       │
       ├──► SuperOdom (existing baseline, untouched)
       │
       └──► LiORF (candidate under qualification)
              │
              ├── /mapping/imu (from spot_imu_adapter)
              ├── config/mapping/extrinsics.yaml (canonical calibration)
              └── maps/liorf/ (candidate output)
```

### Naming Boundaries

| Artifact | Location | Backend-Neutral? |
|----------|----------|-----------------|
| Raw bags | `bags/raw/<date_run>/` | Yes |
| Replay outputs | `bags/replay/<date_run>/` | Yes |
| Extrinsics | `config/mapping/extrinsics.yaml` | Yes |
| IMU adapter | `src/spot_mapping_common/` | Yes |
| Validators | `src/spot_mapping_common/` | Yes |
| Public topics | `/mapping/*` | Yes |
| LiORF bringup | `src/spot_liorf_bringup/` | No |
| LiORF config | `config/liorf/` | No |
| LiORF maps | `maps/liorf/` | No |
| LiORF tmux | `tmux/liorf_mapping/` | No |
| Final maps | `maps/final/` | Promoted |

## Pinned Revision

- **Repo:** `https://github.com/YJZLuckyBoy/liorf.git`
- **Branch:** `ros2`
- **Commit:** `da22fa07be620ff875e2e2af479f66905baa199f` (branch `liorf-ros2`)
- **Loop-closure backend:** ScanContext (`Scancontext.h`, `SCManager`)

### Pre-Tuning Inspection Checklist

- [ ] Clone with `vcs import src < repos/liorf_mapping.repos`
- [ ] Record commit hash in this file
- [ ] Identify loop-closure backend (SC / RING / FALS / ISC)
- [ ] Verify it builds in isolated Humble overlay
- [ ] Record any required patches

## Phase 0: First Evidence

### Prerequisites
```bash
# Platform check
./scripts/check_platform_matrix.sh

# Runtime conflict check
./scripts/preflight_runtime_conflicts.sh
```

### First Replay Procedure
```bash
# 1. Source liorf environment
source scripts/liorf_env.sh

# 2. Launch liorf replay stack (includes the IMU adapter)
ros2 launch spot_liorf_bringup liorf_replay.launch.py \
    imu_input_topic:=/imu

# 3. In another terminal, play a raw bag with clock
ros2 bag play bags/raw/<date_run>/ --clock --rate 0.5
```

### Phase 0 Gate

- [ ] LiORF builds on Humble in isolated overlay
- [ ] First replay produces non-garbage preliminary map
- [ ] Lidar packet loss < 0.5%

**If any gate fails, stop liorf-specific work and evaluate next candidate.**

## Acceptance Gates (Full)

| Gate | Metric | Threshold |
|------|--------|-----------|
| Build | Isolated Humble overlay | Pass/Fail |
| First replay | Non-garbage map | Visual |
| Packet loss | Lidar | < 0.5% |
| IMU rate | Normalized | > 200 Hz |
| Gravity | Rest measurement | 9.81 ± 0.3 m/s² |
| Lidar/IMU skew | Median | < 5 ms |
| Lidar/IMU skew | 99th percentile | < 20 ms |
| Calibration | Repeatability | < 2 cm, 2°, 2 ms |
| Loop closure | Translational | < 0.5 m |
| Loop closure | Yaw | < 3° |
| Multi-pass alignment | Median static error | < 0.2 m |
| ROS1 handoff | AMCL convergence | < 30 s |

## Assumptions

- SuperOdom remains untouched baseline and comparator
- CycloneDDS is the production middleware
- `flat_body` is the default 2D projection frame
- GPS is explicitly disabled in v1
- ROS1 is offline validation only (no live bridge)
- First operational target is Speedway at UT Austin
