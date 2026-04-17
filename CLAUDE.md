# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This is a ROS (catkin) package. Build from the catkin workspace root:

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Build in Release mode is set by default (`CMAKE_BUILD_TYPE "Release"`, `-O3 -Wall -g`). C++11 standard is required.

## Running

**VLP-16 (16-line LiDAR):**
```bash
roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
rosbag play <YOUR_DATASET>.bag
```

**KITTI (HDL-64):**
```bash
roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
roslaunch aloam_velodyne kitti_helper.launch
```

**Docker:**
```bash
cd docker && make build
./run.sh 16   # or ./run.sh 64
```

## Architecture

A-LOAM implements LiDAR Odometry and Mapping (LOAM) using Eigen and Ceres Solver. The system consists of three sequential ROS nodes connected via topics:

```
/velodyne_points (raw sensor input)
        |
        v
[ascanRegistration]  src/scanRegistration.cpp
        | /laser_cloud_sharp, /laser_cloud_less_sharp
        | /laser_cloud_flat, /laser_cloud_less_flat
        | /velodyne_cloud_2 (full resolution)
        v
[alaserOdometry]     src/laserOdometry.cpp
        | /laser_cloud_corner_last, /laser_cloud_surf_last
        | /velodyne_cloud_3, /laser_odom_to_init
        v
[alaserMapping]      src/laserMapping.cpp
        | /laser_cloud_surround, /laser_cloud_map
        | /aft_mapped_to_init (final pose), /laser_after_mapped_path
```

### Node responsibilities

- **scanRegistration** (`ascanRegistration`): Receives raw Velodyne point clouds, organizes points into scan rings, computes per-point curvature, and classifies points as corner (sharp/less-sharp) or planar (flat/less-flat) features. Runs at 10 Hz (scan period = 0.1s).

- **laserOdometry** (`alaserOdometry`): Receives feature point clouds from scan registration and estimates frame-to-frame motion at ~10 Hz. Uses Ceres Solver with point-to-line (corner) and point-to-plane (surface) residuals defined in `lidarFactor.hpp`. Outputs odometry at `/laser_odom_to_init`.

- **laserMapping** (`alaserMapping`): Receives odometry and feature clouds from laserOdometry; refines the pose against a global voxel-grid map at a lower frequency (configurable via `mapping_skip_frame`). Maintains a cube-based local map around the current position and outputs the final corrected pose at `/aft_mapped_to_init`.

- **kittiHelper** (`kittiHelper`): Utility node that reads KITTI dataset files from disk and publishes them as ROS topics (does not participate in the SLAM pipeline itself).

### Key files

- `include/aloam_velodyne/common.h` — `PointType` typedef (`pcl::PointXYZI`) and degree/radian helpers
- `include/aloam_velodyne/tic_toc.h` — `TicToc` class for millisecond-precision timing
- `src/lidarFactor.hpp` — Ceres cost functions: `LidarEdgeFactor` (point-to-line) and `LidarPlaneFactor` (point-to-plane), plus distance factors used in both odometry and mapping

### Key parameters (set in launch files)

| Parameter | Description |
|-----------|-------------|
| `scan_line` | Number of LiDAR scan lines (16 or 64) |
| `minimum_range` | Min distance to keep points (meters) |
| `mapping_skip_frame` | Run mapping every N odometry frames (1 = 10 Hz, 2 = 5 Hz) |
| `mapping_line_resolution` | Voxel size for corner features in map |
| `mapping_plane_resolution` | Voxel size for planar features in map |

### Rotation representation

Poses are stored as a 4-element array `[qx, qy, qz, qw]` (Ceres parametrization) in odometry and as Eigen quaternion + translation vector in mapping. The `lidarFactor.hpp` residuals operate directly on these Ceres parameter blocks.
