#!/bin/bash
# Test script for point cloud filters

echo "======================================"
echo "Testing Point Cloud Filters"
echo "======================================"
echo ""
echo "Make sure your depth camera is running and publishing to:"
echo "  /zed/zed_node/point_cloud/cloud_registered"
echo ""
echo "Press Ctrl+C to stop any running filter"
echo ""

# Source workspace
cd /home/gunreben/ros2_ws
source install/setup.bash

# Check if point cloud is publishing
echo "Checking if point cloud is being published..."
timeout 3 ros2 topic hz /zed/zed_node/point_cloud/cloud_registered 2>&1 | head -5
if [ $? -ne 0 ]; then
    echo ""
    echo "WARNING: No data on /zed/zed_node/point_cloud/cloud_registered"
    echo "Make sure your camera node is running!"
    echo ""
fi

echo ""
echo "======================================"
echo "Test 1: Alpha Filter"
echo "======================================"
echo "Running for 10 seconds..."
echo ""

timeout 10 ros2 run lidar_filter zed_alpha_filter_node \
  --ros-args \
  -p input_topic:=/zed/zed_node/point_cloud/cloud_registered \
  -p output_topic:=/zed/point_cloud/alpha_filtered

echo ""
echo "======================================"
echo "Test 2: Outlier Filter"
echo "======================================"
echo "Running for 10 seconds..."
echo ""

timeout 10 ros2 run lidar_filter outlier_filter_node \
  --ros-args \
  -p input_topic:=/zed/point_cloud/alpha_filtered \
  -p output_topic:=/zed/point_cloud/clean \
  -p voxel_size:=0.05 \
  -p min_points_per_voxel:=2

echo ""
echo "======================================"
echo "Tests Complete!"
echo "======================================"
echo ""
echo "Check output topics:"
echo "  /zed/point_cloud/alpha_filtered  (after alpha filter)"
echo "  /zed/point_cloud/clean           (after outlier filter)"
echo ""
echo "Or run the complete pipeline with:"
echo "  ros2 launch lidar_filter cleanup.launch.py"
echo ""
echo "Visualize in RViz2:"
echo "  rviz2"
echo ""


