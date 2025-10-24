# ROS2 Pointcloud Filter Tools

A comprehensive ROS 2 package providing modular point cloud filtering capabilities for depth cameras and LiDAR sensors. Designed for real-time performance with flexible, chainable filter nodes.

## Overview

This package provides efficient point cloud filtering nodes that can be used individually or chained together for comprehensive point cloud cleanup:

1. **`zed_alpha_filter_node`** - Filters points based on alpha channel (confidence) values
2. **`outlier_filter_node`** ⚡ - Fast voxel-based outlier removal (RECOMMENDED)
3. **`combined_lidar_filter_node`** - Multi-purpose filter for traditional LiDAR (aperture, box, ground plane)

## Quick Start

### Run the Complete Cleanup Pipeline

The easiest way to get started is using the cleanup launch file, which chains the alpha filter and outlier filter:

```bash
# Launch the complete filtering pipeline
ros2 launch lidar_filter cleanup.launch.py

# Input:  /zed/zed_node/point_cloud/cloud_registered
# Output: /zed/zed_node/point_cloud/cloud_registered/filtered
```

This pipeline:
1. Filters low-confidence points using alpha channel
2. Removes outliers using fast voxel-based filtering

### Run Individual Filters

You can also run filters individually for custom pipelines:

```bash
# Alpha channel filter only
ros2 run lidar_filter zed_alpha_filter_node \
  --ros-args \
  -p input_topic:=/zed/zed_node/point_cloud/cloud_registered \
  -p output_topic:=/zed/point_cloud/filtered

# Outlier filter only
ros2 run lidar_filter outlier_filter_node \
  --ros-args \
  -p input_topic:=/zed/point_cloud/filtered \
  -p output_topic:=/zed/point_cloud/clean
```

## Filter Nodes

### 1. Alpha Filter (`zed_alpha_filter_node`)

**Purpose:** Filter points based on alpha channel (confidence) values from depth cameras.

**What it filters:**
- ✅ Low-confidence depth measurements
- ✅ Points with alpha channel not matching threshold

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/zed/zed_node/point_cloud/cloud_registered` | Input point cloud topic |
| `output_topic` | string | `/zed/point_cloud/filtered` | Output point cloud topic |
| `alpha_threshold` | int | 255 | Alpha value to keep (typically 255 for fully opaque) |

**Example usage:**

```bash
# Basic usage with defaults
ros2 run lidar_filter zed_alpha_filter_node

# With custom parameters
ros2 run lidar_filter zed_alpha_filter_node \
  --ros-args \
  -p alpha_threshold:=250
```

---

### 2. Outlier Filter (`outlier_filter_node`) ⚡ RECOMMENDED

**Purpose:** Ultra-fast outlier removal using voxel grid. Best choice for real-time applications.

**What it filters:**
- ✅ NaN and Inf points (automatically)
- ✅ Out-of-range points
- ✅ Isolated points (points in sparse voxels)
- ✅ Scattered artifacts

**How it works:**
1. Divides 3D space into voxels (cubes of size `voxel_size`)
2. Counts points in each voxel
3. Removes points in voxels with fewer than `min_points_per_voxel` points

**Advantages:**
- ⚡ **10-50x faster** than PCL-based filters (5-10ms per frame)
- No KD-tree building (just hash table lookups)
- Works directly on PointCloud2 (no PCL conversion overhead)
- Linear time complexity O(n)
- Real-time capable (30+ Hz on large clouds)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/zed/zed_node/point_cloud/cloud_registered` | Input point cloud topic |
| `output_topic` | string | `/zed/point_cloud/outlier_filtered` | Output point cloud topic |
| `voxel_size` | double | 0.05 | Voxel cube size in meters |
| `min_points_per_voxel` | int | 2 | Min points in voxel to keep |
| `max_valid_range` | double | 100.0 | Max coordinate value in meters |
| `enable_voxel_filter` | bool | true | Enable voxel filtering |

**Example usage:**

```bash
# Default settings (good for most cases)
ros2 run lidar_filter outlier_filter_node

# Aggressive outlier removal
ros2 run lidar_filter outlier_filter_node \
  --ros-args \
  -p voxel_size:=0.03 \
  -p min_points_per_voxel:=3

# Less aggressive (preserve more edge points)
ros2 run lidar_filter outlier_filter_node \
  --ros-args \
  -p voxel_size:=0.1 \
  -p min_points_per_voxel:=1

# Only NaN/Inf/range filtering (no outlier detection)
ros2 run lidar_filter outlier_filter_node \
  --ros-args \
  -p enable_voxel_filter:=false
```

**Tuning tips:**

For **voxel_size**:
- Smaller (0.03m) = catches smaller isolated points, more aggressive
- Larger (0.1m) = only removes very isolated clusters, less aggressive
- Match to scene detail: indoor/close = 0.03-0.05m, outdoor = 0.05-0.1m

For **min_points_per_voxel**:
- Higher (3-5) = more aggressive, removes more outliers
- Lower (1-2) = less aggressive, preserves more edge points
- Use 2 as default (removes true isolated points, keeps surfaces)

---

### 3. Combined LiDAR Filter (`combined_lidar_filter_node`)

**Purpose:** Multi-purpose filter for traditional spinning LiDAR sensors (Ouster, Velodyne, etc.).

**Capabilities:**
- Aperture filtering (field of view restriction)
- Box filtering (remove robot body, specific regions)
- Ground plane removal (RANSAC-based)

See source code for detailed parameter documentation.

---

## Custom Launch Files

Create your own custom filtering pipeline by creating a launch file:

```python
# my_custom_filter.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Stage 1: Alpha filter
        Node(
            package='lidar_filter',
            executable='zed_alpha_filter_node',
            name='alpha_filter',
            parameters=[{
                'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
                'output_topic': '/zed/point_cloud/alpha_filtered',
                'alpha_threshold': 255,
            }],
            output='screen',
        ),
        
        # Stage 2: Outlier filter with custom parameters
        Node(
            package='lidar_filter',
            executable='outlier_filter_node',
            name='outlier_filter',
            parameters=[{
                'input_topic': '/zed/point_cloud/alpha_filtered',
                'output_topic': '/zed/point_cloud/clean',
                'voxel_size': 0.03,
                'min_points_per_voxel': 3,
            }],
            output='screen',
        ),
    ])
```

Run with:
```bash
ros2 launch <your_package> my_custom_filter.launch.py
```

---

## Performance Considerations

### Processing Speed

Approximate processing times (for 640×360 ZED cloud with ~45k valid points):

| Filter | Speed | Notes |
|--------|-------|-------|
| `zed_alpha_filter` | ~5ms | PCL conversion overhead |
| `outlier_filter` ⚡ | ~5-15ms | **RECOMMENDED**, hash table only |

**Real-world example:**
- Outlier filter: 8ms → **125 Hz capable**

### Memory Usage

- Alpha filter: Low (streaming with PCL buffer)
- Outlier filter: Low (hash table only, ~1-2 MB)

### Filter Order Recommendation

For best performance, chain filters in this order:

1. **`zed_alpha_filter_node`** - Removes invalid data quickly
2. **`outlier_filter_node`** - Fast outlier removal on cleaner data

This is exactly what the `cleanup.launch.py` does!

---

## Building the Package

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select lidar_filter

# Source the workspace
source install/setup.bash
```

---

## Visualization

Compare filtered vs unfiltered point clouds in RViz2:

```bash
# Launch your point cloud source
ros2 launch <your_package> <your_launch_file>

# In another terminal, launch the cleanup pipeline
ros2 launch lidar_filter cleanup.launch.py

# Start RViz2
rviz2
```

Add these PointCloud2 displays:
- `/zed/zed_node/point_cloud/cloud_registered` (original, display in red)
- `/zed/point_cloud/alpha_filtered` (after alpha filter, display in yellow)
- `/zed/zed_node/point_cloud/cloud_registered/filtered` (final output, display in green)

---

## Troubleshooting

### "Too many points removed"

If filters are too aggressive:
- Increase `voxel_size` (0.05 → 0.1)
- Decrease `min_points_per_voxel` (2 → 1)

### "Artifacts still visible"

If artifacts remain:
- Decrease `voxel_size` (0.05 → 0.03)
- Increase `min_points_per_voxel` (2 → 3)
- Lower the `alpha_threshold` if using alpha filter

### "Filter is too slow"

To improve performance:
- Use `outlier_filter_node` for fast real-time filtering
- Increase `voxel_size` to process fewer voxels
- Reduce point cloud resolution at the source

---

## Dependencies

- ROS 2 (Humble or later)
- rclcpp
- sensor_msgs
- pcl_ros
- pcl_conversions
- PCL (Point Cloud Library)

---

## License

Apache License 2.0

---

## Contributing

This is a general-purpose point cloud filtering toolbox. Contributions are welcome! Please maintain:
- Clear parameter documentation
- Performance benchmarks for new filters
- Example usage in the README
