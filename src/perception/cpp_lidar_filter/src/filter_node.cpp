#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "cpp_lidar_filter/radial_filter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include "visualization_msgs/msg/marker.hpp"

namespace cpp_lidar_filter
{

class LidarFilterNode : public rclcpp::Node
{
public:
  explicit LidarFilterNode(const rclcpp::NodeOptions & options)
  : Node("lidar_filter_node", options),
    cloud_in_(new pcl::PCLPointCloud2),
    cloud_range_cropped_(new pcl::PCLPointCloud2),
    cloud_body_cropped_(new pcl::PCLPointCloud2),
    cloud_filtered_(new pcl::PCLPointCloud2),
    navigation_indices_(std::make_shared<pcl::Indices>())
  {
    this->declare_parameter("input_topic", "/livox/lidar/pointcloud");
    this->declare_parameter("output_topic", "/livox/lidar_filtered/pointcloud");
    this->declare_parameter("marker_topic", "/livox/lidar_filter/crop_box_marker");
    this->declare_parameter("marker_frame", "base_link");
    this->declare_parameter("navigation_frame", "base_link");
    // 导航范围裁剪参数
    this->declare_parameter("navigation_range", 10.0);
    // 裁剪参数 (CropBox)
    this->declare_parameter("min_x", -0.4);
    this->declare_parameter("max_x", 0.4);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    this->declare_parameter("negative", true); // true = 挖掉中间
    // 降采样参数
    this->declare_parameter("leaf_size", 0.06);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string marker_topic = this->get_parameter("marker_topic").as_string();
    marker_frame_ = this->get_parameter("marker_frame").as_string();
    navigation_frame_ = this->get_parameter("navigation_frame").as_string();
    navigation_range_ = this->get_parameter("navigation_range").as_double();
    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    negative_ = this->get_parameter("negative").as_bool();
    leaf_size_ = this->get_parameter("leaf_size").as_double();

    if (!std::isfinite(navigation_range_) || navigation_range_ <= 0.0) {
      throw std::invalid_argument("navigation_range must be finite and greater than zero");
    }
    if (!std::isfinite(leaf_size_) || leaf_size_ < 0.05 || leaf_size_ > 0.08) {
      throw std::invalid_argument("leaf_size must be finite and between 0.05 and 0.08 metres");
    }
    if (navigation_frame_.empty()) {
      throw std::invalid_argument("navigation_frame must not be empty");
    }

    navigation_extract_.setIndices(navigation_indices_);
    navigation_extract_.setNegative(false);

    body_min_pt_ << min_x_, min_y_, min_z_, 1.0;
    body_max_pt_ << max_x_, max_y_, max_z_, 1.0;
    body_crop_.setMin(body_min_pt_);
    body_crop_.setMax(body_max_pt_);
    body_crop_.setNegative(negative_);

    const float leaf_size = static_cast<float>(leaf_size_);
    voxel_.setLeafSize(leaf_size, leaf_size, leaf_size);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Navigation crop: %.1f m in %s, voxel leaf size: %.2f m",
      navigation_range_, navigation_frame_.c_str(), leaf_size_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void publish_marker()
  {
    if (marker_pub_->get_subscription_count() == 0) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_;
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 计算中心点位置
    marker.pose.position.x = (max_x_ + min_x_) / 2.0;
    marker.pose.position.y = (max_y_ + min_y_) / 2.0;
    marker.pose.position.z = (max_z_ + min_z_) / 2.0;
    marker.pose.orientation.w = 1.0;

    // 计算尺寸
    marker.scale.x = max_x_ - min_x_;
    marker.scale.y = max_y_ - min_y_;
    marker.scale.z = max_z_ - min_z_;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4; //半透明

    marker_pub_->publish(marker);
  }
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (pub_->get_subscription_count() + pub_->get_intra_process_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        navigation_frame_, msg->header.frame_id, msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Dropping point cloud because transform %s <- %s is unavailable: %s",
        navigation_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const auto & translation = transform_stamped.transform.translation;
    const auto & rotation = transform_stamped.transform.rotation;
    const Eigen::Affine3d navigation_from_cloud =
      Eigen::Translation3d(translation.x, translation.y, translation.z) *
      Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);

    pcl_conversions::toPCL(*msg, *cloud_in_);

    // 先做精确水平半径裁剪，避免远距离点进入后续处理。
    if (!collectHorizontalRangeIndices(
        *cloud_in_, navigation_range_, navigation_from_cloud, *navigation_indices_))
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Point cloud must use packed little-endian FLOAT32/FLOAT64 x, y, and z fields");
      return;
    }
    navigation_extract_.setInputCloud(cloud_in_);
    navigation_extract_.filter(*cloud_range_cropped_);

    // 去除车身点。
    body_crop_.setInputCloud(cloud_range_cropped_);
    body_crop_.filter(*cloud_body_cropped_);

    // 体素降采样，减少地面分割输入点数。
    voxel_.setInputCloud(cloud_body_cropped_);
    voxel_.filter(*cloud_filtered_);

    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_filtered_, output);
    output.header = msg->header;

    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  pcl::ExtractIndices<pcl::PCLPointCloud2> navigation_extract_;
  pcl::CropBox<pcl::PCLPointCloud2> body_crop_;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_;
  pcl::PCLPointCloud2::Ptr cloud_in_;
  pcl::PCLPointCloud2::Ptr cloud_range_cropped_;
  pcl::PCLPointCloud2::Ptr cloud_body_cropped_;
  pcl::PCLPointCloud2::Ptr cloud_filtered_;
  pcl::IndicesPtr navigation_indices_;
  Eigen::Vector4f body_min_pt_;
  Eigen::Vector4f body_max_pt_;
  std::string marker_frame_;
  std::string navigation_frame_;
  double navigation_range_{10.0};
  double min_x_{-0.4};
  double max_x_{0.4};
  double min_y_{-0.3};
  double max_y_{0.3};
  double min_z_{-0.1};
  double max_z_{0.6};
  double leaf_size_{0.06};
  bool negative_{true};
};

}  // namespace cpp_lidar_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cpp_lidar_filter::LidarFilterNode)
