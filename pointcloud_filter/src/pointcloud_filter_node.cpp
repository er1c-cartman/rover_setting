#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct AngleFilter {
  double min_angle;
  double max_angle;
};

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter(const rclcpp::NodeOptions & options)
  : Node("pointcloud_filter_node", options)
  {
    this->declare_parameter<std::string>("input_topic", "input_pointcloud");
    this->declare_parameter<std::string>("output_topic", "filtered_pointcloud");
    this->declare_parameter<std::vector<double>>("angle_filters", {});
    this->declare_parameter<double>("distance_threshold", 0.1);
    this->declare_parameter<double>("max_distance", 20.0); // New parameter for max distance

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    max_distance_ = this->get_parameter("max_distance").as_double(); // Get max distance parameter

    // Load angle filters
    std::vector<double> angle_filters_vec = this->get_parameter("angle_filters").as_double_array();
    for (size_t i = 0; i < angle_filters_vec.size(); i += 2) {
      if (i + 1 < angle_filters_vec.size()) {
        angle_filters_.emplace_back(AngleFilter{angle_filters_vec[i], angle_filters_vec[i + 1]});
      }
    }

    // Add debugging statements
    RCLCPP_INFO(this->get_logger(), "Loaded input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Loaded output_topic: %s", output_topic_.c_str());
    for (const auto& filter : angle_filters_) {
      RCLCPP_INFO(this->get_logger(), "Loaded angle filter: [%f, %f]", filter.min_angle, filter.max_angle);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded distance threshold: %f", distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "Loaded max distance: %f", max_distance_);

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", output_topic_.c_str());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud message");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Apply angle filters and distance threshold
    pcl::PointCloud<pcl::PointXYZ>::Ptr angle_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    angle_filtered_cloud->points.reserve(cloud->points.size());
    for (const auto& point : cloud->points) {
      double distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance < distance_threshold_ || distance > max_distance_) { // Use max distance parameter
        continue;
      }
      
      double angle = std::atan2(point.y, point.x);
      bool in_filter = false;
      for (const auto& filter : angle_filters_) {
        if (angle >= filter.min_angle && angle <= filter.max_angle) {
          in_filter = true;
          break;
        }
      }
      if (!in_filter) {
        angle_filtered_cloud->points.push_back(point);
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toPCLPointCloud2(*angle_filtered_cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, output);
    output.header.frame_id = msg->header.frame_id; // Set the frame ID
    output.header.stamp = msg->header.stamp; // Set the timestamp
    publisher_->publish(output);

    RCLCPP_INFO(this->get_logger(), "Published filtered point cloud message");
  }

  std::string input_topic_;
  std::string output_topic_;
  double distance_threshold_;
  double max_distance_; // New member variable for max distance
  std::vector<AngleFilter> angle_filters_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // No parameter overrides here to ensure it reads from the YAML file

  auto node = std::make_shared<PointCloudFilter>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

