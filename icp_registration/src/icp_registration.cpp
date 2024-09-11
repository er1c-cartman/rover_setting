#include "icp_registration/icp_registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
      first_scan_(true) {
  is_ready_ = false;
  cloud_in_ = pcl::make_shared<PointCloudXYZ>();

  // Reading the lidar roll correction from parameters
  double lidar_roll_correction = this->declare_parameter("lidar_roll_correction", 0.0);
  
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }

  // Read the pcd file
  pcl::PCDReader reader;
  PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
  reader.read(pcd_path_, *cloud);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  refine_map_ = cloud;  // Directly assign the cloud after filtering

  PointCloudXYZ::Ptr point_rough(new PointCloudXYZ);
  voxel_rough_filter_.setInputCloud(refine_map_);
  voxel_rough_filter_.filter(*point_rough);

  rough_map_ = point_rough;

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  range_odom_frame_id_ =
      this->declare_parameter("range_odom_frame_id", std::string("lidar_odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("livox_frame"));
  thresh_ = this->declare_parameter("thresh", 1.0);
  xy_offset_ = this->declare_parameter("xy_offset", 0.5);
  yaw_offset_ = this->declare_parameter("yaw_offset", 60.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 5.0) * M_PI / 180.0;
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);

    // Apply roll correction from the configuration
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3) + lidar_roll_correction, 
             initial_pose_vec.at(4),
             initial_pose_vec.at(5));
    initial_pose_.orientation = tf2::toMsg(q);
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));

  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                     "Publishing tf"
                       << map_to_odom_.transform.translation.x << " "
                       << map_to_odom_.transform.translation.y << " "
                       << map_to_odom_.transform.translation.z);
          map_to_odom_.header.stamp = now();
          map_to_odom_.header.frame_id = map_frame_id_;
          map_to_odom_.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(map_to_odom_);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  if (tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::fromROSMsg(*msg, *cloud_in_);
  if (first_scan_) {
    auto pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose.pose = initial_pose_;
    initialPoseCallback(pose_msg);
    first_scan_ = false;
  }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix4d initial_guess;
  initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
  initial_guess.block<3, 1>(0, 3) = pos;
  initial_guess(3, 3) = 1;

  RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");
  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
  if (!success_) {
    map_to_laser = initial_guess;
    RCLCPP_ERROR(this->get_logger(), "ICP failed");
  }

  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    auto transform =
        tf_buffer_->lookupTransform(laser_frame_id_, range_odom_frame_id_, now(),
                                    rclcpp::Duration::from_seconds(10));
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
    laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    is_ready_ = false;
    return;
  }
  Eigen::Matrix4d result = map_to_laser * laser_to_odom.matrix().cast<double>();
  {
    std::lock_guard lock(mutex_);
    map_to_odom_.transform.translation.x = result(0, 3);
    map_to_odom_.transform.translation.y = result(1, 3);
    map_to_odom_.transform.translation.z = result(2, 3);

    Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
    q = Eigen::Quaterniond(rotation);

    map_to_odom_.transform.rotation.w = q.w();
    map_to_odom_.transform.rotation.x = q.x();
    map_to_odom_.transform.rotation.y = q.y();
    map_to_odom_.transform.rotation.z = q.z();
    is_ready_ = true;
  }
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZ::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -yaw_offset_; k <= yaw_offset_; k++) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Generated %zu candidates for rough alignment", candidates.size());

  PointCloudXYZ::Ptr rough_source(new PointCloudXYZ);
  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);

  PointCloudXYZ::Ptr align_point(new PointCloudXYZ);

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  auto tic = std::chrono::system_clock::now();

  for (Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.setInputSource(rough_source);
    icp_rough_.align(*align_point, init_pose);
    double rough_score = icp_rough_.getFitnessScore();
    RCLCPP_INFO(this->get_logger(), "Rough alignment score: %f", rough_score);

    if (!icp_rough_.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "ICP rough alignment did not converge.");
      continue;
    }
    if (rough_score > 2 * thresh_) {
      RCLCPP_WARN(this->get_logger(), "Rough alignment score too high, skipping candidate.");
      continue;
    }
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  if (!rough_converge) {
    RCLCPP_ERROR(this->get_logger(), "No suitable rough alignment candidate found.");
    return Eigen::Matrix4d::Zero();
  }

  icp_refine_.setInputSource(source);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();
  RCLCPP_INFO(this->get_logger(), "Refined alignment score: %f", score_);

  if (!icp_refine_.hasConverged()) {
    RCLCPP_ERROR(this->get_logger(), "ICP refine alignment did not converge.");
    return Eigen::Matrix4d::Zero();
  }
  if (score_ > thresh_) {
    RCLCPP_ERROR(this->get_logger(), "Refined alignment score above threshold.");
    return Eigen::Matrix4d::Zero();
  }

  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "ICP alignment completed in: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "Final ICP score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
