#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "cpp_lidar_filter/single_pass_filter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace cpp_lidar_filter
{
namespace
{

Eigen::Affine3d affineFromTransform(const geometry_msgs::msg::Transform & transform)
{
  const auto & translation = transform.translation;
  const auto & rotation = transform.rotation;
  return Eigen::Translation3d(translation.x, translation.y, translation.z) *
         Eigen::Quaterniond(rotation.w, rotation.x, rotation.y, rotation.z);
}

}  // namespace

class LidarFilterNode : public rclcpp::Node
{
public:
  explicit LidarFilterNode(const rclcpp::NodeOptions & options) : Node("lidar_filter_node", options)
  {
    this->declare_parameter("input_topic", "/livox/lidar/pointcloud");
    this->declare_parameter("output_topic", "/livox/lidar_filtered/pointcloud");
    this->declare_parameter("marker_topic", "/livox/lidar_filter/crop_box_marker");
    this->declare_parameter("marker_frame", "base_link");
    this->declare_parameter("navigation_frame", "base_link");
    // 导航范围裁剪参数
    this->declare_parameter("navigation_range", 10.0);
    // 云台自车结构裁剪盒（navigation=base_link=雷达系；z 相对雷达平面）
    this->declare_parameter("min_x", -0.4);
    this->declare_parameter("max_x", 0.4);
    this->declare_parameter("min_y", -0.3);
    this->declare_parameter("max_y", 0.3);
    this->declare_parameter("min_z", -0.1);
    this->declare_parameter("max_z", 0.6);
    this->declare_parameter("negative", true);  // true = 挖掉中间
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
      this->get_logger(), "Navigation crop: %.1f m in %s, voxel leaf size: %.2f m", params_.range,
      navigation_frame_.c_str(), params_.leaf);
    // 云台自车结构裁剪盒的坐标系语义提醒（只打日志，不改判定）：本工作区
    // small_glim 的 base_link 实际携带雷达位姿（见 small_glim/config/params_node.yaml:38），
    // 且雷达本身装在云台上，所以这个盒子的 z 是「相对雷达平面」的高度，
    // 用来挖掉同云台上的其他结构（支架/相机/线缆/云台臂），不是底盘盒。
    RCLCPP_INFO(
      this->get_logger(),
      "Gimbal self-structure crop box x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] "
      "evaluated in '%s' (negative=%d). base_link carries the lidar pose and the lidar "
      "is mounted on the gimbal, so this box removes other gimbal-mounted structure; "
      "z is relative to the lidar plane.",
      params_.body_min_x, params_.body_max_x, params_.body_min_y, params_.body_max_y,
      params_.body_min_z, params_.body_max_z, navigation_frame_.c_str(),
      static_cast<int>(params_.negative));
    if (!marker_frame_.empty() && marker_frame_ != navigation_frame_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Crop-box marker_frame '%s' differs from navigation_frame '%s'; marker pose "
        "will be transformed via TF (falls back to %s if unavailable).",
        marker_frame_.c_str(), navigation_frame_.c_str(), navigation_frame_.c_str());
    }

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());
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

    // 裁剪判定与 box 几何都在 navigation_frame_ 里。marker 若配置到别的 frame，
    // 必须把 box 位姿显式变换过去：旧实现只改 header.frame_id、位置仍按
    // navigation_frame 的数值解释，换 frame 后 marker 整体错位（看起来像裁剪盒
    // 在别处，误导排查）。两者相同（默认）时不查 TF，零开销。
    std::string marker_frame = navigation_frame_;
    Eigen::Affine3d marker_from_navigation = Eigen::Affine3d::Identity();
    if (!marker_frame_.empty() && marker_frame_ != navigation_frame_) {
      try {
        const auto tf =
          tf_buffer_->lookupTransform(marker_frame_, navigation_frame_, tf2::TimePointZero);
        marker_from_navigation = affineFromTransform(tf.transform);
        marker_frame = marker_frame_;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Cannot transform crop box %s -> %s for marker (%s); publishing it in %s.",
          navigation_frame_.c_str(), marker_frame_.c_str(), ex.what(), navigation_frame_.c_str());
      }
    }

    // box 中心在 navigation_frame 下的坐标（与裁剪判定同系同语义）。
    Eigen::Affine3d navigation_from_box = Eigen::Affine3d::Identity();
    navigation_from_box.translation() = Eigen::Vector3d(
      (params_.body_max_x + params_.body_min_x) / 2.0,
      (params_.body_max_y + params_.body_min_y) / 2.0,
      (params_.body_max_z + params_.body_min_z) / 2.0);
    const Eigen::Affine3d marker_from_box = marker_from_navigation * navigation_from_box;
    const Eigen::Quaterniond marker_orientation =
      Eigen::Quaterniond(marker_from_box.linear()).normalized();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame;
    marker.header.stamp = this->now();
    marker.ns = "gimbal_self_structure";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = marker_from_box.translation().x();
    marker.pose.position.y = marker_from_box.translation().y();
    marker.pose.position.z = marker_from_box.translation().z();
    marker.pose.orientation.x = marker_orientation.x();
    marker.pose.orientation.y = marker_orientation.y();
    marker.pose.orientation.z = marker_orientation.z();
    marker.pose.orientation.w = marker_orientation.w();

    // 计算尺寸
    marker.scale.x = params_.body_max_x - params_.body_min_x;
    marker.scale.y = params_.body_max_y - params_.body_min_y;
    marker.scale.z = params_.body_max_z - params_.body_min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;  //半透明

    marker_pub_->publish(marker);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (pub_->get_subscription_count() + pub_->get_intra_process_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped =
        tf_buffer_->lookupTransform(navigation_frame_, msg->header.frame_id, msg->header.stamp);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Dropping point cloud because transform %s <- %s is unavailable: %s",
        navigation_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return;
    }

    const Eigen::Affine3d navigation_from_cloud = affineFromTransform(transform_stamped.transform);

    // 单遍：变换判定（半径 + 云台自车结构 box）+ 近似体素降采样一次完成。
    // 输出点坐标仍留在输入系、header 保持输入帧，下游地面分割语义不变。
    if (!filterSinglePass(*msg, navigation_from_cloud, params_, filtered_cloud_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Dropping point cloud: x/y/z must be FLOAT32/FLOAT64 fields with a valid "
        "row_step and point_step layout");
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
