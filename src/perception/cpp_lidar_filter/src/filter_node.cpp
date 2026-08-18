#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "cpp_lidar_filter/single_pass_filter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <pcl_conversions/pcl_conversions.h>
#include "visualization_msgs/msg/marker.hpp"

namespace cpp_lidar_filter
{

class LidarFilterNode : public rclcpp::Node
{
public:
  explicit LidarFilterNode(const rclcpp::NodeOptions & options)
  : Node("lidar_filter_node", options)
  {
    this->declare_parameter("input_topic", "/livox/lidar/pointcloud");
    this->declare_parameter("output_topic", "/livox/lidar_filtered/pointcloud");
    this->declare_parameter("marker_topic", "/livox/lidar_filter/crop_box_marker");
    this->declare_parameter("marker_frame", "base_link");
    this->declare_parameter("navigation_frame", "base_link");
    // 导航范围裁剪参数
    this->declare_parameter("navigation_range", 10.0);
    // 裁剪参数 (车身 box，navigation 系)
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
    params_.range = this->get_parameter("navigation_range").as_double();
    params_.body_min_x = this->get_parameter("min_x").as_double();
    params_.body_max_x = this->get_parameter("max_x").as_double();
    params_.body_min_y = this->get_parameter("min_y").as_double();
    params_.body_max_y = this->get_parameter("max_y").as_double();
    params_.body_min_z = this->get_parameter("min_z").as_double();
    params_.body_max_z = this->get_parameter("max_z").as_double();
    params_.negative = this->get_parameter("negative").as_bool();
    params_.leaf = this->get_parameter("leaf_size").as_double();

    if (!std::isfinite(params_.range) || params_.range <= 0.0) {
      throw std::invalid_argument("navigation_range must be finite and greater than zero");
    }
    if (!std::isfinite(params_.leaf) || params_.leaf < 0.05 || params_.leaf > 0.08) {
      throw std::invalid_argument("leaf_size must be finite and between 0.05 and 0.08 metres");
    }
    if (navigation_frame_.empty()) {
      throw std::invalid_argument("navigation_frame must not be empty");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(), "Navigation crop: %.1f m in %s, voxel leaf size: %.2f m",
      params_.range, navigation_frame_.c_str(), params_.leaf);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());
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

    // 计算中心点位置（与裁剪判定同系：navigation_frame）
    marker.pose.position.x = (params_.body_max_x + params_.body_min_x) / 2.0;
    marker.pose.position.y = (params_.body_max_y + params_.body_min_y) / 2.0;
    marker.pose.position.z = (params_.body_max_z + params_.body_min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    // 计算尺寸
    marker.scale.x = params_.body_max_x - params_.body_min_x;
    marker.scale.y = params_.body_max_y - params_.body_min_y;
    marker.scale.z = params_.body_max_z - params_.body_min_z;

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

    // 单遍：变换判定（半径 + 车身 box）+ 近似体素降采样一次完成。
    // 输出点坐标仍留在输入系、header 保持输入帧，下游地面分割语义不变。
    if (!filterSinglePass(*msg, navigation_from_cloud, params_, filtered_cloud_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Point cloud must use packed little-endian FLOAT32/FLOAT64 x, y, and z fields");
      return;
    }

    // 用 unique_ptr 发布：与 ground_segmentation 同容器且开启 intra-process 时，
    // rclcpp 直接把所有权转成 shared_ptr 投递给进程内订阅者，整帧点云不再经过
    // DDS 序列化/反序列化，也不会为 const& publish 再做一次堆上深拷贝。
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(filtered_cloud_, *output);
    output->header = msg->header;

    pub_->publish(std::move(output));
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  SinglePassParams params_;
  pcl::PointCloud<pcl::PointXYZI> filtered_cloud_;  // 复用缓冲，避免每帧重分配
  std::string marker_frame_;
  std::string navigation_frame_;
};

}  // namespace cpp_lidar_filter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cpp_lidar_filter::LidarFilterNode)
