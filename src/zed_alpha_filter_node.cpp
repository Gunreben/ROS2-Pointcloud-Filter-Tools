#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ZedAlphaFilterNode : public rclcpp::Node
{
public:
  ZedAlphaFilterNode()
  : Node("zed_alpha_filter_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("output_topic", "/zed/point_cloud/filtered");
    this->declare_parameter<int>("alpha_threshold", 255);  // Default to 255 for 8-bit alpha
    
    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    alpha_threshold_ = this->get_parameter("alpha_threshold").as_int();
    
    // Set up publisher and subscriber with sensor data QoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, sensor_qos);
    
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&ZedAlphaFilterNode::pointCloudCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "ZED Alpha Filter Node initialized");
    RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Alpha threshold: %d (keeping only points with alpha == %d)", 
                alpha_threshold_, alpha_threshold_);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    // Convert ROS2 message to PCL PointCloud with RGBA
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Create filtered cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    filtered_cloud->points.reserve(cloud->points.size());
    
    // Filter points based on alpha channel
    size_t removed_count = 0;
    for (const auto& point : cloud->points) {
      // Check if alpha channel equals the threshold (typically 255 for fully opaque)
      if (point.a == alpha_threshold_) {
        filtered_cloud->points.push_back(point);
      } else {
        removed_count++;
      }
    }
    
    // Update cloud properties
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = cloud->is_dense;
    filtered_cloud->header = cloud->header;
    
    // Log filtering statistics periodically
    if (frame_count_ % 30 == 0) {  // Every ~1 second at 30 Hz
      RCLCPP_INFO(this->get_logger(), 
                  "Filtered cloud: kept %zu/%zu points (removed %zu points with alpha != %d)",
                  filtered_cloud->points.size(), cloud->points.size(), removed_count, alpha_threshold_);
    }
    frame_count_++;
    
    // Convert back to ROS message and publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = cloud_msg->header;
    
    pub_->publish(output_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  int alpha_threshold_;
  size_t frame_count_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedAlphaFilterNode>());
  rclcpp::shutdown();
  return 0;
}

