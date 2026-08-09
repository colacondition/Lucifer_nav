#include "grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation2
{

class RmPathSmoother : public rclcpp::Node
{
public:
  explicit RmPathSmoother(const rclcpp::NodeOptions & options)
  : Node("rm_path_smoother", options)
  {
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "/plan_raw");
    output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/plan");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    smoothing_iterations_ = declare_parameter<int>("smoothing_iterations", 40);
    data_weight_ = declare_parameter<double>("data_weight", 0.25);
    smooth_weight_ = declare_parameter<double>("smooth_weight", 0.35);
    tolerance_ = declare_parameter<double>("tolerance", 0.00001);
    use_costmap_constraints_ = declare_parameter<bool>("use_costmap_constraints", true);
    obstacle_cost_threshold_ = declare_parameter<int>("obstacle_cost_threshold", 50);
    max_cost_increase_ = declare_parameter<int>("max_cost_increase", 5);
    preserve_turn_angle_ = declare_parameter<double>("preserve_turn_angle", 0.25);
    turn_smoothing_scale_ = declare_parameter<double>("turn_smoothing_scale", 0.25);

    // 输入是粗路径，输出是平滑后的路径。
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        smoothAndPublish(*msg, "planner");
      });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = std::move(msg);
      });
    path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, rclcpp::QoS(1).reliable());

    RCLCPP_INFO(
      get_logger(), "rm_path_smoother ready: planner=%s -> %s",
      input_path_topic_.c_str(), output_path_topic_.c_str());
  }

private:
  bool costmapCompatible(const nav_msgs::msg::Path & path) const
  {
    // 没有代价图时直接放行。
    if (!use_costmap_constraints_ || !costmap_) {
      return false;
    }
    return path.header.frame_id.empty() || costmap_->header.frame_id.empty() ||
           path.header.frame_id == costmap_->header.frame_id;
  }

  bool pointTraversable(double world_x, double world_y) const
  {
    // 单点是否落在可通行区。
    if (!costmap_) {
      return true;
    }

    int map_x = 0;
    int map_y = 0;
    if (!worldToMap(*costmap_, world_x, world_y, map_x, map_y)) {
      return false;
    }

    const auto value = costmap_->data[gridIndex(*costmap_, map_x, map_y)];
    return !isOccupied(value, obstacle_cost_threshold_, true);
  }

  int pointCost(double world_x, double world_y) const
  {
    if (!costmap_) {
      return 0;
    }

    int map_x = 0;
    int map_y = 0;
    if (!worldToMap(*costmap_, world_x, world_y, map_x, map_y)) {
      return 100;
    }

    return static_cast<int>(costmap_->data[gridIndex(*costmap_, map_x, map_y)]);
  }

  double turnAngle(const nav_msgs::msg::Path & path, std::size_t index) const
  {
    if (index == 0 || index + 1 >= path.poses.size()) {
      return 0.0;
    }

    const auto & prev = path.poses[index - 1].pose.position;
    const auto & curr = path.poses[index].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    const double a0 = std::atan2(curr.y - prev.y, curr.x - prev.x);
    const double a1 = std::atan2(next.y - curr.y, next.x - curr.x);
    return std::abs(normalizeAngle(a1 - a0));
  }

  bool segmentTraversable(double start_x, double start_y, double end_x, double end_y) const
  {
    // 线段上的格子都得安全。
    if (!costmap_) {
      return true;
    }

    int start_map_x = 0;
    int start_map_y = 0;
    int end_map_x = 0;
    int end_map_y = 0;
    if (!worldToMap(*costmap_, start_x, start_y, start_map_x, start_map_y) ||
      !worldToMap(*costmap_, end_x, end_y, end_map_x, end_map_y))
    {
      return false;
    }

    const auto cells = raytraceLine(start_map_x, start_map_y, end_map_x, end_map_y);
    for (const auto & cell : cells) {
      const auto value = costmap_->data[gridIndex(*costmap_, cell.x, cell.y)];
      if (isOccupied(value, obstacle_cost_threshold_, true)) {
        return false;
      }
    }
    return true;
  }

  bool candidateTraversable(
    const nav_msgs::msg::Path & path, std::size_t index, double candidate_x, double candidate_y,
    int original_cost) const
  {
    if (!costmapCompatible(path) || index == 0 || index + 1 >= path.poses.size()) {
      return true;
    }

    if (pointCost(candidate_x, candidate_y) > original_cost + max_cost_increase_) {
      return false;
    }

    const auto & prev = path.poses[index - 1].pose.position;
    const auto & next = path.poses[index + 1].pose.position;
    return pointTraversable(candidate_x, candidate_y) &&
           segmentTraversable(prev.x, prev.y, candidate_x, candidate_y) &&
           segmentTraversable(candidate_x, candidate_y, next.x, next.y);
  }

  void updateOrientations(nav_msgs::msg::Path & path) const
  {
    // 按相邻点重新写航向。
    if (path.poses.empty()) {
      return;
    }
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const auto & current = path.poses[i].pose.position;
      const auto & next = path.poses[i + 1].pose.position;
      const double yaw = std::atan2(next.y - current.y, next.x - current.x);
      path.poses[i].pose.orientation = quaternionFromYaw(yaw);
    }
    if (path.poses.size() >= 2) {
      path.poses.back().pose.orientation = path.poses[path.poses.size() - 2].pose.orientation;
    }
  }

  void smoothAndPublish(const nav_msgs::msg::Path & input, const char * source_label)
  {
    // 点太少时不平滑，直接透传。
    if (input.poses.size() < 3 || smoothing_iterations_ <= 0) {
      nav_msgs::msg::Path passthrough = input;
      passthrough.header.stamp = now();
      updateOrientations(passthrough);
      path_pub_->publish(passthrough);
      RCLCPP_DEBUG(
        get_logger(), "Published %s path without smoothing: %zu poses",
        source_label, passthrough.poses.size());
      return;
    }

    nav_msgs::msg::Path output = input;
    output.header.stamp = now();

    std::vector<double> original_x;
    std::vector<double> original_y;
    std::vector<int> original_costs;
    original_x.reserve(output.poses.size());
    original_y.reserve(output.poses.size());
    original_costs.reserve(output.poses.size());
    for (const auto & pose : output.poses) {
      original_x.push_back(pose.pose.position.x);
      original_y.push_back(pose.pose.position.y);
      original_costs.push_back(pointCost(pose.pose.position.x, pose.pose.position.y));
    }

    for (int iteration = 0; iteration < smoothing_iterations_; ++iteration) {
      double change = 0.0;
      for (std::size_t i = 1; i + 1 < output.poses.size(); ++i) {
        auto & point = output.poses[i].pose.position;
        const double old_x = point.x;
        const double old_y = point.y;
        const double curve = turnAngle(output, i);
        const double corner_scale =
          curve >= preserve_turn_angle_ ? turn_smoothing_scale_ :
          std::clamp(1.0 - curve / std::max(1e-6, preserve_turn_angle_), turn_smoothing_scale_, 1.0);
        const double local_data_weight =
          curve >= preserve_turn_angle_ ? data_weight_ * 1.25 : data_weight_;
        const double local_smooth_weight = smooth_weight_ * corner_scale;

        point.x += local_data_weight * (original_x[i] - point.x);
        point.y += local_data_weight * (original_y[i] - point.y);
        point.x += local_smooth_weight *
          (output.poses[i - 1].pose.position.x + output.poses[i + 1].pose.position.x -
          2.0 * point.x);
        point.y += local_smooth_weight *
          (output.poses[i - 1].pose.position.y + output.poses[i + 1].pose.position.y -
          2.0 * point.y);

        if (!candidateTraversable(output, i, point.x, point.y, original_costs[i])) {
          point.x = old_x;
          point.y = old_y;
          continue;
        }

        change += std::abs(old_x - point.x) + std::abs(old_y - point.y);
      }
      if (change < tolerance_) {
        break;
      }
    }

    updateOrientations(output);
    path_pub_->publish(output);
    RCLCPP_DEBUG(
      get_logger(), "Smoothed %s path: %zu poses -> %s", source_label, output.poses.size(),
      output_path_topic_.c_str());
  }

  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string costmap_topic_;
  int smoothing_iterations_{40};
  double data_weight_{0.25};
  double smooth_weight_{0.35};
  double tolerance_{0.00001};
  bool use_costmap_constraints_{true};
  int obstacle_cost_threshold_{50};
  int max_cost_increase_{5};
  double preserve_turn_angle_{0.25};
  double turn_smoothing_scale_{0.25};

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmPathSmoother)
