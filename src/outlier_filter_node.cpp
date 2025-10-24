#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>
#include <unordered_map>

class OutlierFilterNode : public rclcpp::Node
{
public:
  OutlierFilterNode()
  : Node("outlier_filter_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("output_topic", "/zed/point_cloud/outlier_filtered");
    this->declare_parameter<double>("voxel_size", 0.05);           // Voxel grid size (m)
    this->declare_parameter<int>("min_points_per_voxel", 2);       // Min points in voxel to keep
    this->declare_parameter<double>("max_valid_range", 100.0);     // Max coordinate value
    this->declare_parameter<bool>("enable_voxel_filter", true);    // Enable voxel-based filtering
    
    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    min_points_per_voxel_ = this->get_parameter("min_points_per_voxel").as_int();
    max_valid_range_ = this->get_parameter("max_valid_range").as_double();
    enable_voxel_filter_ = this->get_parameter("enable_voxel_filter").as_bool();
    
    // Precompute inverse voxel size
    inv_voxel_size_ = 1.0 / voxel_size_;
    
    // Set up publisher and subscriber with sensor data QoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
    
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&OutlierFilterNode::pointCloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Outlier Filter Node initialized");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Voxel filter: %s", enable_voxel_filter_ ? "enabled" : "disabled");
    if (enable_voxel_filter_) {
      RCLCPP_INFO(this->get_logger(), "Voxel size: %.3fm, min points: %d", 
                  voxel_size_, min_points_per_voxel_);
    }
  }

private:
  // Fast voxel coordinate hashing
  struct VoxelKey {
    int x, y, z;
    
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  
  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
      // Simple hash combination
      return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
    }
  };
  
  inline VoxelKey getVoxelKey(float x, float y, float z) const {
    return VoxelKey{
      static_cast<int>(std::floor(x * inv_voxel_size_)),
      static_cast<int>(std::floor(y * inv_voxel_size_)),
      static_cast<int>(std::floor(z * inv_voxel_size_))
    };
  }
  
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Statistics
    size_t input_count = 0;
    size_t nan_inf_count = 0;
    size_t range_filtered = 0;
    size_t outlier_filtered = 0;
    
    // Create iterators
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    // First pass: collect valid points and build voxel grid
    struct PointData {
      float x, y, z;
      const uint8_t* data_ptr;
      VoxelKey voxel_key;
    };
    std::vector<PointData> valid_points;
    valid_points.reserve(msg->width * msg->height / 2);
    
    std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_counts;
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      input_count++;
      
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;
      
      // Check for NaN/Inf
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        nan_inf_count++;
        continue;
      }
      
      // Range check
      float max_coord = std::max({std::abs(x), std::abs(y), std::abs(z)});
      if (max_coord > max_valid_range_) {
        range_filtered++;
        continue;
      }
      
      PointData pd;
      pd.x = x;
      pd.y = y;
      pd.z = z;
      pd.data_ptr = reinterpret_cast<const uint8_t*>(&(*iter_x));
      
      if (enable_voxel_filter_) {
        pd.voxel_key = getVoxelKey(x, y, z);
        voxel_counts[pd.voxel_key]++;
      }
      
      valid_points.push_back(pd);
    }
    
    // Second pass: filter by voxel occupancy
    auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    output_msg->header = msg->header;
    output_msg->height = 1;
    output_msg->width = 0;
    output_msg->fields = msg->fields;
    output_msg->is_bigendian = msg->is_bigendian;
    output_msg->point_step = msg->point_step;
    output_msg->is_dense = false;
    output_msg->data.reserve(valid_points.size() * msg->point_step);
    
    for (const auto& pd : valid_points) {
      bool keep = true;
      
      if (enable_voxel_filter_) {
        // Check if voxel has enough neighbors
        int voxel_count = voxel_counts[pd.voxel_key];
        if (voxel_count < min_points_per_voxel_) {
          outlier_filtered++;
          keep = false;
        }
      }
      
      if (keep) {
        output_msg->data.insert(output_msg->data.end(), 
                                pd.data_ptr, 
                                pd.data_ptr + msg->point_step);
        output_msg->width++;
      }
    }
    
    output_msg->row_step = output_msg->width * output_msg->point_step;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Log statistics periodically
    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Filtered: kept %u/%zu points | removed %zu NaN/Inf, %zu range, %zu outliers | %ld ms",
                  output_msg->width, input_count, nan_inf_count, range_filtered, 
                  outlier_filtered, duration.count());
    }
    frame_count_++;
    
    pub_->publish(*output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  double voxel_size_;
  double inv_voxel_size_;
  int min_points_per_voxel_;
  double max_valid_range_;
  bool enable_voxel_filter_;
  size_t frame_count_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutlierFilterNode>());
  rclcpp::shutdown();
  return 0;
}

