cmake_minimum_required(VERSION 3.8)
project(lidar_filter)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# ament_target_dependencies() was removed in Lyrical: link the exported
# targets (plus include dirs, for older header-only exports) directly.
function(link_ros_deps target)
  foreach(dep ${ARGN})
    if(dep STREQUAL "PCL")
      continue()
    endif()
    target_link_libraries(${target} ${${dep}_TARGETS} ${${dep}_LIBRARIES})
    target_include_directories(${target} PRIVATE ${${dep}_INCLUDE_DIRS})
  endforeach()
endfunction()
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS 
  common 
  io 
  filters 
  segmentation
)
# PCL 1.12 (Humble/jammy) does not define Eigen3::Eigen itself
find_package(Eigen3 REQUIRED NO_MODULE)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Define common dependencies for all nodes
set(LIDAR_FILTER_DEPS
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  tf2_ros
  tf2_eigen
  PCL
)

# Combined filter node
add_executable(combined_lidar_filter_node src/combined_lidar_filter_node.cpp)
link_ros_deps(combined_lidar_filter_node
  ${LIDAR_FILTER_DEPS}
  visualization_msgs
)
target_link_libraries(combined_lidar_filter_node ${PCL_LIBRARIES})

# ZED alpha filter node
add_executable(zed_alpha_filter_node src/zed_alpha_filter_node.cpp)
link_ros_deps(zed_alpha_filter_node
  ${LIDAR_FILTER_DEPS}
)
target_link_libraries(zed_alpha_filter_node ${PCL_LIBRARIES})

# Outlier filter node (no PCL, very fast)
add_executable(outlier_filter_node src/outlier_filter_node.cpp)
link_ros_deps(outlier_filter_node
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  combined_lidar_filter_node
  zed_alpha_filter_node
  outlier_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()