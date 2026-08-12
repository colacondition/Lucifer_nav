#include "grid_utils.hpp"
#include "rc_esdf.h"
#include "minco/minco_optimizer.hpp"
#include "semantic_map_consumer.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation2
{

class RmMincoPathSmoother : public rclcpp::Node
{
public:
  explicit RmMincoPathSmoother(const rclcpp::NodeOptions & options)
  : Node("rm_minco_path_smoother", options)
  {
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "/plan_raw");
    output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/plan");

    // MINCO 参数
    smooth_weight_ = declare_parameter<double>("smooth_weight", 1.0);
    obstacle_weight_ = declare_parameter<double>("obstacle_weight", 10.0);
    data_weight_ = declare_parameter<double>("data_weight", 10.0);
    robot_radius_ = declare_parameter<double>("robot_radius", 1.0);
    penalty_mu_ = declare_parameter<double>("penalty_mu", 0.4);
    enable_optimization_ = declare_parameter<bool>("enable_optimization", true);
    // 隧道内偏离轴线的软代价权重。收不到语义地图或图里没隧道时自动失效。
    tunnel_axis_weight_ = declare_parameter<double>("tunnel_axis_weight", 5.0);
    semantic_map_topic_ = declare_parameter<std::string>(
      "semantic_map_topic", "/map_server/semantic_map");

    // 时间分配参数
    default_velocity_ = declare_parameter<double>("default_velocity", 1.0);
    min_segment_time_ = declare_parameter<double>("min_segment_time", 0.1);

    // RC-ESDF 参数
    esdf_width_ = declare_parameter<double>("esdf_width", 2.0);
    esdf_height_ = declare_parameter<double>("esdf_height", 2.0);
    esdf_resolution_ = declare_parameter<double>("esdf_resolution", 0.05);

    // 机器人外形（用于生成 RC-ESDF）
    auto footprint = declare_parameter<std::vector<double>>(
      "robot_footprint",
      std::vector<double>{-0.3, -0.25, 0.3, -0.25, 0.3, 0.25, -0.3, 0.25});

    // 解析机器人外形
    robot_polygon_.clear();
    for (size_t i = 0; i + 1 < footprint.size(); i += 2) {
      robot_polygon_.emplace_back(footprint[i], footprint[i + 1]);
    }

    // 初始化 RC-ESDF
    esdf_map_.initialize(esdf_width_, esdf_height_, esdf_resolution_);
    esdf_map_.generateFromPolygon(robot_polygon_);

    // 初始化 MINCO 优化器
    minco_optimizer_ = std::make_unique<MincoOptimizer>();
    MincoOptimizer::Params params;
    params.smooth_weight = smooth_weight_;
    params.obstacle_weight = obstacle_weight_;
    params.data_weight = data_weight_;
    params.robot_radius = robot_radius_;
    params.penalty_mu = penalty_mu_;
    params.tunnel_axis_weight = tunnel_axis_weight_;
    params.enable = enable_optimization_;
    minco_optimizer_->setParams(params);

    // 设置 ESDF 查询函数
    minco_optimizer_->setEsdfQuery(
      [this](const Eigen::Vector2d & pos, double & dist, Eigen::Vector2d & grad) {
        return esdf_map_.query(pos, dist, grad);
      });

    // 隧道轴线查询：给 MINCO 的对齐软代价用。语义地图收不到时 receiver_.map() 无效，
    // tunnelAxisAtPoint 恒返回 false，对齐项自动失效。
    minco_optimizer_->setTunnelAxisQuery(
      [this](const Eigen::Vector2d & pos, Eigen::Vector2d & axis) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        return tunnelAxisAtPoint(receiver_.map(), pos, axis);
      });

    // 语义地图和代价地图同源同 QoS。收不到时对齐项静默失效（隧道被当普通空地），
    // 不报错。整帧不自洽时保留上一张好图。
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, map_qos,
      [this](decision_interfaces::msg::SemanticMap::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex_);
        try {
          receiver_.update(*msg);
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
        }
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        smoothAndPublish(*msg);
      });

    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, rclcpp::QoS(1).reliable());

    RCLCPP_INFO(
      get_logger(), "rm_minco_path_smoother ready: %s -> %s",
      input_path_topic_.c_str(), output_path_topic_.c_str());
  }

private:
  void smoothAndPublish(const nav_msgs::msg::Path & input_path)
  {
    if (input_path.poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "Path too short for MINCO smoothing");
      path_pub_->publish(input_path);
      return;
    }

    // 提取路径点
    std::vector<Eigen::Vector2d> waypoints;
    waypoints.reserve(input_path.poses.size());
    for (const auto & pose : input_path.poses) {
      waypoints.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // 计算段时间（基于路径点间距离）
    std::vector<double> segment_times;
    segment_times.reserve(waypoints.size() - 1);
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      double dist = (waypoints[i + 1] - waypoints[i]).norm();
      double time = std::max(dist / default_velocity_, min_segment_time_);
      segment_times.push_back(time);
    }

    // MINCO 优化
    auto start_time = now();
    auto pieces = minco_optimizer_->optimize(waypoints, segment_times);
    auto duration = (now() - start_time).seconds();

    if (pieces.empty()) {
      RCLCPP_ERROR(get_logger(), "MINCO optimization failed, publishing original path");
      path_pub_->publish(input_path);
      return;
    }

    RCLCPP_DEBUG(get_logger(), "MINCO optimization took %.3f ms", duration * 1000.0);

    // 采样轨迹到路径
    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;

    double sample_dt = 0.1;  // 每 0.1 秒采样一次
    for (size_t i = 0; i < pieces.size(); ++i) {
      const auto & piece = pieces[i];
      double duration = piece.getDuration();
      int num_samples = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));

      for (int j = 0; j < num_samples; ++j) {
        double t = (j * duration) / (num_samples - 1);
        Eigen::Vector2d pos = piece.getPos(t);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = input_path.header;
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        output_path.poses.push_back(pose);
      }
    }

    path_pub_->publish(output_path);
    RCLCPP_DEBUG(
      get_logger(), "Published MINCO path with %zu poses", output_path.poses.size());
  }

  std::string input_path_topic_;
  std::string output_path_topic_;

  double smooth_weight_;
  double obstacle_weight_;
  double data_weight_;
  double robot_radius_;
  double penalty_mu_;
  double tunnel_axis_weight_;
  bool enable_optimization_;
  std::string semantic_map_topic_;

  double default_velocity_;
  double min_segment_time_;

  double esdf_width_;
  double esdf_height_;
  double esdf_resolution_;

  std::vector<Eigen::Vector2d> robot_polygon_;
  RcEsdfMap esdf_map_;

  std::unique_ptr<MincoOptimizer> minco_optimizer_;

  // 语义地图。轴线查询在优化回调里读，语义地图在订阅回调里写，两者可能不同线程
  // （单容器多线程执行器），用锁护住 receiver_。
  SemanticMapReceiver receiver_;
  std::mutex map_mutex_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace navigation2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMincoPathSmoother)
