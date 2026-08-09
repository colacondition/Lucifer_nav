#include <cmath>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

namespace linefit_ground_segmentation {

class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  // 雷达安装高度不再由 yaml 手填，而是从 URDF 外参的 TF 里查出来，见
  // resolveSensorHeight()。空字符串表示关闭该行为、退回 sensor_height 参数。
  std::string sensor_height_frame_;
  // sensor_height_frame_ 原点离地的高度。默认 0：实测 RMUL.pcd 的地面主峰在
  // -0.17，正好等于外参里的雷达安装高度 0.175，说明 base_link 原点基本就在地面
  // 上。底盘换结构、base_link 原点抬到轮轴上时才需要填。
  double base_to_ground_z_{0.0};
  bool sensor_height_resolved_{false};

private:
  void resolveSensorHeight(const std::string &cloud_frame);
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {
  gravity_aligned_frame_ =
      this->declare_parameter("gravity_aligned_frame", "gravity_aligned");

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments =
      this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line =
      this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold =
      this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height =
      this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height =
      this->declare_parameter("max_start_height", params_.max_start_height);
  // sensor_height 只是收不到 TF 时的兜底值。真值从 sensor_height_frame ->
  // 点云 frame 的 TF 里取（见 resolveSensorHeight），这样雷达离地高度在整个仓库
  // 里只有 URDF 外参一处来源：config/{reality,simulation}/measurement_params_*.yaml。
  // 以前这个值要跟外参手工同步，实车就同步失败了 —— 外参 z 填 0.0、
  // 这里填 0.59、xacro 的 default 又是 0.49，三个数互相矛盾。
  params_.sensor_height =
      this->declare_parameter("sensor_height", params_.sensor_height);
  sensor_height_frame_ =
      this->declare_parameter("sensor_height_frame", std::string("base_link"));
  base_to_ground_z_ =
      this->declare_parameter("base_to_ground_z", base_to_ground_z_);
  params_.line_search_angle =
      this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  // Params that need to be squared.
  const double r_min =
      this->declare_parameter("r_min", std::sqrt(params_.r_min_square));
  const double r_max =
      this->declare_parameter("r_max", std::sqrt(params_.r_max_square));
  const double max_fit_error = this->declare_parameter(
      "max_fit_error", std::sqrt(params_.max_error_square));
  if (!std::isfinite(r_min) || r_min < 0.0) {
    throw std::invalid_argument(
        "r_min must be finite and greater than or equal to zero");
  }
  if (!std::isfinite(r_max) || r_max <= r_min) {
    throw std::invalid_argument("r_max must be finite and greater than r_min");
  }
  if (!std::isfinite(max_fit_error) || max_fit_error < 0.0) {
    throw std::invalid_argument(
        "max_fit_error must be finite and greater than or equal to zero");
  }
  params_.r_min_square = r_min * r_min;
  params_.r_max_square = r_max * r_max;
  params_.max_error_square = max_fit_error * max_fit_error;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic =
      this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SegmentationNode::scanCallback, this, std::placeholders::_1));
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic, rclcpp::SensorDataQoS());
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic, rclcpp::SensorDataQoS());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  sensor_height_resolved_ = sensor_height_frame_.empty();
  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
}

// 把 sensor_height 换成从 TF 查出来的真值：
//   sensor_height = z(sensor_height_frame -> 点云 frame) + base_to_ground_z
// 前一项就是 URDF 外参里的雷达安装高度，后一项是 base_link 原点离地的高度
// （底盘几何，换雷达位置不会变）。这样「雷达装多高」在整个仓库里只有
// measurement_params_{real,sim}.yaml 一处来源，real 和 sim 走同一条逻辑。
// 只在第一次成功时重建 segmenter：外参是静态的，查到一次就够。
void SegmentationNode::resolveSensorHeight(const std::string &cloud_frame) {
  if (sensor_height_resolved_ || cloud_frame.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(sensor_height_frame_, cloud_frame,
                                             tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Cannot look up %s -> %s for sensor_height, keep the "
                         "configured fallback %.3f: %s",
                         sensor_height_frame_.c_str(), cloud_frame.c_str(),
                         params_.sensor_height, ex.what());
    return;
  }

  const double mount_z = tf_stamped.transform.translation.z;
  const double resolved = mount_z + base_to_ground_z_;
  if (!std::isfinite(resolved) || resolved <= 0.0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Resolved sensor_height %.3f (mount %.3f + base_to_ground "
                 "%.3f) is not a positive finite number. Check the URDF "
                 "extrinsic in measurement_params_*.yaml. Keep %.3f.",
                 resolved, mount_z, base_to_ground_z_, params_.sensor_height);
    sensor_height_resolved_ = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "sensor_height resolved from TF: %.3f (mount %.3f above %s + "
              "%.3f base_to_ground_z), configured fallback was %.3f",
              resolved, mount_z, sensor_height_frame_.c_str(),
              base_to_ground_z_, params_.sensor_height);
  params_.sensor_height = resolved;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  sensor_height_resolved_ = true;
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const bool publish_ground = ground_pub_->get_subscription_count() > 0;
  const bool publish_obstacle = obstacle_pub_->get_subscription_count() > 0;
  if (!publish_ground && !publish_obstacle) {
    return;
  }

  // 外参是静态 TF，开机后头几帧可能还没进 buffer，所以在回调里查、查到即锁定。
  resolveSensorHeight(msg->header.frame_id);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

  std::vector<int> labels;

  bool is_original_pc = true;
  if (!gravity_aligned_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf;
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> &cloud_proc =
      is_original_pc ? cloud : cloud_transformed;

  segmenter_->segment(cloud_proc, &labels);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  if (publish_ground) {
    ground_cloud.reserve(cloud.size());
  }
  if (publish_obstacle) {
    obstacle_cloud.reserve(cloud.size());
  }
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (labels[i] == 1) {
      if (publish_ground) {
        ground_cloud.push_back(cloud[i]);
      }
    } else if (publish_obstacle) {
      obstacle_cloud.push_back(cloud[i]);
    }
  }
  if (publish_ground) {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(ground_cloud, ground_msg);
    ground_msg.header = msg->header;
    ground_pub_->publish(ground_msg);
  }
  if (publish_obstacle) {
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(obstacle_cloud, obstacle_msg);
    obstacle_msg.header = msg->header;
    obstacle_pub_->publish(obstacle_msg);
  }
}

}  // namespace linefit_ground_segmentation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(linefit_ground_segmentation::SegmentationNode)
