# ROS2 Pointcloud Filter Tools (`lidar_filter`)

Modular ROS 2 point cloud filter nodes, chainable per topic. Package
name: **`lidar_filter`** (repo name differs — clone into your workspace
and build with `colcon build --packages-select lidar_filter`).

Three independent nodes:

| Node | Purpose | Typical input |
|------|---------|---------------|
| `combined_lidar_filter_node` | Aperture (FOV) + box (robot body) + RANSAC ground removal for spinning lidars | Ouster/Velodyne clouds |
| `outlier_filter_node` | Fast voxel-hash outlier removal, O(n), no PCL trees | any PointCloud2 |
| `zed_alpha_filter_node` | Drop low-confidence points via the alpha channel of ZED RGBA clouds | ZED `cloud_registered` |

## combined_lidar_filter_node

The workhorse for the tractor stack: feeds `obstacle_detection` with a
cleaned cloud (`/ouster/points` → `/ouster/points/filtered`).

Processing order per frame:
1. Optional transform into `target_frame` (TF; skipped with a warning if
   the transform is unavailable — the output header always states the
   true frame).
2. **Aperture filter** — keep points inside an azimuth/elevation window.
   Azimuth is measured around **+Y = forward** (matches this rig's
   mesh-derived frame convention; see note below).
3. **Box filter** — `pcl::CropBox`; with `box_filter_negative: true`
   (default) it *removes* the box interior, i.e. the vehicle's own body.
4. **Ground filter** — single-plane RANSAC (z-axis constrained by
   `angle_threshold`), removes inliers.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `ouster/points` | PointCloud2 input |
| `output_topic` | `ouster/points/filtered` | PointCloud2 output |
| `target_frame` | `base_link` | Transform target (empty = keep input frame) |
| `enable_aperture_filter` | true | |
| `min_azimuth_angle` / `max_azimuth_angle` | −20 / 20 | deg, around +Y forward |
| `min_elevation_angle` / `max_elevation_angle` | −45 / 45 | deg |
| `enable_box_filter` | true | |
| `min_point` / `max_point` | tractor body box | `[x, y, z]` in `target_frame` (m) |
| `box_filter_negative` | true | true = remove inside (self-filter) |
| `enable_ground_filter` | true | |
| `distance_threshold` | 0.2 | RANSAC inlier distance (m) |
| `max_iterations` | 100 | RANSAC iterations |
| `angle_threshold` | 15.0 | max plane tilt vs z-axis (deg) |
| `optimize_coefficients` | true | |

All parameters are runtime-tunable (`ros2 param set …`); changes apply
immediately, including derived values (angle radians) and filters that
were disabled at startup.

Debug outputs: `filter_box_marker` and `aperture_filter_marker`
(`visualization_msgs/Marker`, published at 2 Hz when the respective
filter is enabled).

**Frame convention note:** on the vario700 rig the TF tree follows the
Farming-Simulator-derived mesh orientation (y ≈ forward). The aperture
filter's azimuth reference assumes this. If you use this node on a
REP-103 rig (x forward), adapt the azimuth math or window accordingly.

### Performance

The ground RANSAC dominates runtime on large clouds (~131k-point Ouster
frames). On embedded targets consider `max_iterations: 25–50`, or run
the box/aperture filters first (they shrink the cloud before RANSAC —
this is the built-in order already).

## outlier_filter_node

Ultra-fast outlier removal using a voxel hash (no KD-tree, no PCL
conversion): removes NaN/Inf, out-of-range coordinates, and points in
sparsely populated voxels. ~5–15 ms on a ~45k-point cloud.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/zed/zed_node/point_cloud/cloud_registered` | |
| `output_topic` | `/zed/point_cloud/outlier_filtered` | |
| `voxel_size` | 0.05 | m; smaller = more aggressive |
| `min_points_per_voxel` | 2 | higher = more aggressive |
| `max_valid_range` | 100.0 | m, coordinate sanity bound |
| `enable_voxel_filter` | true | false = only NaN/Inf/range cleanup |

Tuning: indoor/close range `voxel_size` 0.03–0.05, outdoor 0.05–0.1.

## zed_alpha_filter_node

Keeps only points whose alpha (confidence) channel equals
`alpha_threshold` (default 255). Only meaningful for RGBA clouds.

| Parameter | Default |
|-----------|---------|
| `input_topic` | `/zed/zed_node/point_cloud/cloud_registered` |
| `output_topic` | `/zed/point_cloud/filtered` |
| `alpha_threshold` | 255 |

## Launch files

- `cleanup.launch.py` — ZED cleanup chain: alpha filter → outlier
  filter (`/zed/zed_node/point_cloud/cloud_registered` →
  `…/cloud_registered/filtered`).
- For the tractor obstacle-detection stack, `combined_lidar_filter_node`
  is launched by `obstacle_detection`'s
  `safety_zone_visualizer_stack.launch.py`.

Example standalone run:

```bash
ros2 run lidar_filter combined_lidar_filter_node --ros-args \
  -p input_topic:=/ouster/points \
  -p output_topic:=/ouster/points/filtered \
  -p enable_aperture_filter:=false
```

## Dependencies

ROS 2 Humble+, `rclcpp`, `sensor_msgs`, `visualization_msgs`,
`pcl_ros`, `pcl_conversions`, `tf2_ros`, `tf2_eigen`, PCL.

## Changelog

### 2026-07
- Runtime parameter updates fixed: the set-parameters callback now
  applies the *incoming* values (previously it re-read the old values,
  so every change lagged one call behind).
- All filter parameters are loaded at startup regardless of enable
  flags (enabling a filter at runtime no longer starts from
  uninitialized members).
- README rewritten: documented `combined_lidar_filter_node` (previously
  "see source code"), corrected the package/repo naming and removed the
  outdated ZED-only framing.

## License

Apache License 2.0
