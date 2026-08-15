#include "distance_transform.hpp"
#include "grid_utils.hpp"
#include "path_stitching.hpp"
#include "semantic_map_consumer.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace navigation2
{

namespace
{

// 清障场的缓存键：地图内容（FNV-1a）+ 阈值语义 + 分辨率 + 尺寸。
std::uint64_t clearanceFieldHash(
  const nav_msgs::msg::OccupancyGrid & grid, int threshold, bool unknown_is_obstacle)
{
  std::uint64_t h = 14695981039346656037ULL;
  for (const auto byte : grid.data) {
    h ^= static_cast<std::uint8_t>(byte);
    h *= 1099511628211ULL;
  }
  h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(threshold)) * 0x9E3779B97F4A7C15ULL;
  h ^= unknown_is_obstacle ? 0x7F4A7C15C4B45C3EULL : 0x3B64AD1F0E62D1A7ULL;
  std::uint64_t resolution_bits = 0;
  std::memcpy(&resolution_bits, &grid.info.resolution, sizeof(resolution_bits));
  h ^= resolution_bits + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
  h ^= static_cast<std::uint64_t>(grid.info.width) * 0xC2B2AE3D27D4EB4FULL;
  h ^= static_cast<std::uint64_t>(grid.info.height) * 0x165667B19E3779F9ULL;
  return h;
}

}  // namespace

class RmGlobalPlanner : public rclcpp::Node
{
public:
  explicit RmGlobalPlanner(const rclcpp::NodeOptions & options)
  : Node("rm_global_planner", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    // 本节点跑在 component_container_mt（多线程 executor）里，而所有订阅回调
    // 和定时器回调都读写同一批成员（map_、goal_、last_path_、清障缓存、
    // 语义地图轴线表）并可能触发同步 A*。全部放进同一个互斥回调组，让
    // rclcpp 串行化这些回调——这比手写锁更不容易漏（此前 plan_mtx_ 声明了
    // 却从未加锁，goal/map/timer 回调在 mt 容器下真会并发）。
    planner_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = planner_callback_group_;

    // 地图是静态输入，用 transient_local 直接补最新值。
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        const bool had_map = static_cast<bool>(map_);
        map_ = std::move(msg);
        if (!had_map) {
          RCLCPP_INFO(
            get_logger(), "Received planning grid %ux%u from %s", map_->info.width,
            map_->info.height, map_topic_.c_str());
        }
        if (goal_ && (!had_map || replan_on_source_update_)) {
          planFromCurrentPose(true);
        }
      },
      sub_options);

    // 语义地图和代价地图同源同 QoS，收不到时行为退回纯几何 A*（隧道被当普通空地，
    // 斜切进洞口不会被拦），不报错。
    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, map_qos,
      [this](decision_interfaces::msg::SemanticMap::ConstSharedPtr msg) {
        try {
          if (!receiver_.update(*msg)) {
            return;
          }
        } catch (const std::exception & ex) {
          // 整帧拒收：通道长度对不上意味着格号可能整体错位，拿错位的轴线去判方向
          // 会在错误的位置放行斜切。保留上一张好图。
          RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
          return;
        }
        // 轴线表按代价地图的格号索引，语义地图换了就得重建。
        axis_grid_built_ = false;
        RCLCPP_INFO(
          get_logger(), "Global planner got semantic map: %zu tunnels",
          receiver_.map().tunnels().size());
        if (goal_) {
          planFromCurrentPose(true);
        }
      },
      sub_options);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        goal_ = *msg;
        last_path_.reset();
        last_planned_start_.reset();
        // 目标改变 → 代次自增。发布前会再对比，防止慢 A* 的结果覆盖新目标。
        ++plan_gen_;
        RCLCPP_INFO(
          get_logger(), "Received goal at (%.2f, %.2f) in frame %s",
          goal_->pose.position.x, goal_->pose.position.y, goal_->header.frame_id.c_str());
        planFromCurrentPose(true);
      },
      sub_options);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).reliable());
    replan_sub_ = create_subscription<std_msgs::msg::Empty>(
      replan_request_topic_, rclcpp::QoS(1),
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        if (goal_) {
          RCLCPP_INFO(get_logger(), "Received explicit replan request");
          // 明确重规划 → 代次自增，确保此次结果不被旧的慢请求覆盖。
          ++plan_gen_;
          planFromCurrentPose(true);
        }
      },
      sub_options);

    // 定时重规划作为兜底。
    if (planning_frequency_ > 0.0 && replan_on_timer_) {
      const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, planning_frequency_));
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          if (goal_) {
            planFromCurrentPose(false);
          }
        },
        planner_callback_group_);
    }

    RCLCPP_INFO(
      get_logger(), "rm_global_planner ready: %s -> %s", goal_topic_.c_str(),
      path_topic_.c_str());
  }

  ~RmGlobalPlanner() = default;

private:
  struct QueueItem
  {
    int index{0};
    double priority{0.0};
    bool operator>(const QueueItem & other) const { return priority > other.priority; }
  };

  void loadParameters()
  {
    // 规划框架和代价参数。
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/plan_raw");
    planning_frequency_ = declare_parameter<double>("planning_frequency", 2.0);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", true);
    unknown_cost_ = declare_parameter<double>("unknown_cost", 1.4);
    obstacle_threshold_ = declare_parameter<int>("obstacle_threshold", 50);
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.25);
    apply_internal_inflation_ = declare_parameter<bool>("apply_internal_inflation", true);
    map_cost_weight_ = declare_parameter<double>("map_cost_weight", 10.0);
    map_cost_exponent_ = declare_parameter<double>("map_cost_exponent", 2.0);
    planner_tolerance_ = declare_parameter<double>("planner_tolerance", 0.5);
    use_clearance_cost_ = declare_parameter<bool>("use_clearance_cost", true);
    clearance_desired_distance_ = declare_parameter<double>("clearance_desired_distance", 0.6);
    clearance_cost_weight_ = declare_parameter<double>("clearance_cost_weight", 3.0);
    replan_on_timer_ = declare_parameter<bool>("replan_on_timer", true);
    replan_on_source_update_ = declare_parameter<bool>("replan_on_source_update", false);
    min_replan_distance_ = declare_parameter<double>("min_replan_distance", 0.2);
    replan_request_topic_ = declare_parameter<std::string>(
      "replan_request_topic", "/navigation2/replan_request");
    turn_penalty_ = declare_parameter<double>("turn_penalty", 0.2);
    local_stitch_enabled_ = declare_parameter<bool>("local_stitch_enabled", true);
    stitch_min_lookahead_distance_ =
      declare_parameter<double>("stitch_min_lookahead_distance", 0.8);
    stitch_max_distance_ = declare_parameter<double>("stitch_max_distance", 3.0);
    path_prune_enabled_ = declare_parameter<bool>("path_prune_enabled", true);
    prune_segment_max_cost_ = declare_parameter<int>("prune_segment_max_cost", 75);
    path_resample_distance_ = declare_parameter<double>("path_resample_distance", 0.18);
    inflation_cost_scaling_factor_ =
      declare_parameter<double>("inflation_cost_scaling_factor", 8.0);
    // 规划失败后的冷却时间。目标不可达时规划器会在 planning_frequency 的频率
    // 反复重试，浪费算力且洪水般地刷日志。冷却期内如果目标没有明显变化就跳过，
    // 等待环境更新（新代价图、障碍移动）后再重试。
    plan_failure_cooldown_ = declare_parameter<double>("plan_failure_cooldown", 2.0);
    // 路径发布前的可行性验收：沿路径采样检查代价地图，拒绝穿越障碍的路径。
    // path_smoother 已有代价约束，这里主要防止极端情况下 A* 拼接尾段时引入
    // 已变成障碍的段落。
    path_acceptance_enabled_ = declare_parameter<bool>("path_acceptance_enabled", true);
    path_acceptance_max_cost_ = declare_parameter<int>("path_acceptance_max_cost", 85);

    semantic_map_topic_ =
      declare_parameter<std::string>("semantic_map_topic", "/map_server/semantic_map");
    // 隧道内偏离轴线的代价权重：每走一步加 w * (1 - |cos θ|)，θ 是步进方向与隧道
    // 轴线的夹角。设 0 关闭。
    //
    // 洞只比车稍宽，横向余量很小，斜着走一格就贴壁 —— 这个代价把路径压到轴线上。
    //
    // 为什么是软代价而不是硬性禁止斜步：车体是圆柱，朝向不影响能不能过，只要对着
    // 洞口进去就行，「偏离轴线」是该少走的，不是不能走。硬阈值还会在洞口附近把可行
    // 的一步整个拿掉，逼出绕路或不可达。
    //
    // 量级：单格代价基线是 1.0，权重 2.0 时垂直于轴线的一步要 2.0，绕开它去走沿轴
    // 的两三格更便宜 —— 塑形够强，但不会让洞变成不可达。
    tunnel_axis_cost_weight_ = declare_parameter<double>("tunnel_axis_cost_weight", 2.0);
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      pose.header.frame_id = global_frame_;
      pose.header.stamp = now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot get robot pose %s -> %s: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool isTraversable(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
  {
    if (!inBounds(grid, x, y)) {
      return false;
    }
    const auto value = grid.data[gridIndex(grid, x, y)];
    return !isOccupied(value, obstacle_threshold_, !allow_unknown_);
  }

  double cellCost(const nav_msgs::msg::OccupancyGrid & grid, int x, int y) const
  {
    const auto value = grid.data[gridIndex(grid, x, y)];
    if (value < 0) {
      return std::max(1.0, unknown_cost_);
    }
    const double normalized_cost =
      std::clamp(static_cast<double>(std::max<int>(0, value)) / 100.0, 0.0, 1.0);
    return 1.0 + map_cost_weight_ * std::pow(normalized_cost, map_cost_exponent_);
  }

  std::optional<int> nearestTraversableIndex(
    const nav_msgs::msg::OccupancyGrid & grid, int requested_x, int requested_y,
    int max_radius_cells) const
  {
    // 先用原点，不行再找最近可通行格。
    if (isTraversable(grid, requested_x, requested_y)) {
      return static_cast<int>(gridIndex(grid, requested_x, requested_y));
    }

    double best_dist_sq = std::numeric_limits<double>::infinity();
    std::optional<int> best_index;
    for (int radius = 1; radius <= max_radius_cells; ++radius) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (std::abs(dx) != radius && std::abs(dy) != radius) {
            continue;
          }
          const int x = requested_x + dx;
          const int y = requested_y + dy;
          if (!isTraversable(grid, x, y)) {
            continue;
          }
          const double dist_sq = static_cast<double>(dx * dx + dy * dy);
          if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_index = static_cast<int>(gridIndex(grid, x, y));
          }
        }
      }
      if (best_index) {
        return best_index;
      }
    }
    return std::nullopt;
  }

  std::vector<int> extractPathIndices(
    const std::vector<int> & parent, int start_index, int goal_index) const
  {
    std::vector<int> indices;
    int current = goal_index;
    while (current >= 0 && current != start_index) {
      indices.push_back(current);
      current = parent[static_cast<std::size_t>(current)];
    }
    indices.push_back(start_index);
    std::reverse(indices.begin(), indices.end());
    return indices;
  }

  bool isSegmentTraversable(
    const nav_msgs::msg::OccupancyGrid & grid, int start_index, int end_index) const
  {
    // 线段上的每个格子都不能碰障。
    const int start_x = start_index % static_cast<int>(grid.info.width);
    const int start_y = start_index / static_cast<int>(grid.info.width);
    const int end_x = end_index % static_cast<int>(grid.info.width);
    const int end_y = end_index / static_cast<int>(grid.info.width);

    const auto cells = raytraceLine(start_x, start_y, end_x, end_y);
    for (const auto & cell : cells) {
      if (!isTraversable(grid, cell.x, cell.y)) {
        return false;
      }
    }
    return true;
  }

  int segmentMaxCost(
    const nav_msgs::msg::OccupancyGrid & grid, int start_index, int end_index) const
  {
    // 记录这段路径经过的最高代价。
    const int start_x = start_index % static_cast<int>(grid.info.width);
    const int start_y = start_index / static_cast<int>(grid.info.width);
    const int end_x = end_index % static_cast<int>(grid.info.width);
    const int end_y = end_index / static_cast<int>(grid.info.width);

    int max_cost = -1;
    for (const auto & cell : raytraceLine(start_x, start_y, end_x, end_y)) {
      if (!inBounds(grid, cell.x, cell.y)) {
        return 100;
      }
      const int value = static_cast<int>(grid.data[gridIndex(grid, cell.x, cell.y)]);
      if (value > max_cost) {
        max_cost = value;
      }
    }
    return max_cost;
  }

  bool segmentIsSafeForPrune(
    const nav_msgs::msg::OccupancyGrid & grid, int start_index, int end_index) const
  {
    const int max_cost = segmentMaxCost(grid, start_index, end_index);
    if (max_cost < 0) {
      return false;
    }
    if (isOccupied(static_cast<int8_t>(max_cost), obstacle_threshold_, !allow_unknown_)) {
      return false;
    }
    return max_cost <= prune_segment_max_cost_;
  }

  std::vector<int> prunePathIndices(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<int> & indices) const
  {
    if (!path_prune_enabled_ || indices.size() < 3) {
      return indices;
    }

    std::vector<int> pruned;
    pruned.reserve(indices.size());
    pruned.push_back(indices.front());

    std::size_t anchor = 0;
    while (anchor + 1 < indices.size()) {
      std::size_t best = anchor + 1;
      for (std::size_t candidate = anchor + 2; candidate < indices.size(); ++candidate) {
        if (segmentIsSafeForPrune(grid, indices[anchor], indices[candidate])) {
          best = candidate;
        }
      }
      pruned.push_back(indices[best]);
      anchor = best;
    }

    return pruned;
  }

  std::vector<geometry_msgs::msg::Point> buildWorldPoints(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<int> & indices,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) const
  {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(indices.size() + 2);
    points.push_back(start_pose.pose.position);

    for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
      const int index = indices[i];
      const int x = index % static_cast<int>(grid.info.width);
      const int y = index / static_cast<int>(grid.info.width);

      geometry_msgs::msg::Point point;
      mapToWorld(grid, x, y, point.x, point.y);
      point.z = 0.0;
      points.push_back(point);
    }

    points.push_back(goal_pose.pose.position);
    return points;
  }

  std::vector<geometry_msgs::msg::Point> resampleWorldPoints(
    const std::vector<geometry_msgs::msg::Point> & input) const
  {
    if (path_resample_distance_ <= 0.0 || input.size() < 2) {
      return input;
    }

    std::vector<geometry_msgs::msg::Point> output;
    output.reserve(input.size() * 2);
    output.push_back(input.front());

    double carry = 0.0;
    for (std::size_t i = 1; i < input.size(); ++i) {
      geometry_msgs::msg::Point segment_start = input[i - 1];
      const auto & segment_end = input[i];
      double remaining_segment = std::hypot(
        segment_end.x - segment_start.x, segment_end.y - segment_start.y);
      if (remaining_segment <= 1e-6) {
        continue;
      }

      while (carry + remaining_segment >= path_resample_distance_) {
        const double step = path_resample_distance_ - carry;
        const double ratio = std::clamp(step / remaining_segment, 0.0, 1.0);

        geometry_msgs::msg::Point point;
        point.x = segment_start.x + (segment_end.x - segment_start.x) * ratio;
        point.y = segment_start.y + (segment_end.y - segment_start.y) * ratio;
        point.z = 0.0;
        output.push_back(point);

        segment_start = point;
        remaining_segment = std::hypot(
          segment_end.x - segment_start.x, segment_end.y - segment_start.y);
        carry = 0.0;
      }

      carry += remaining_segment;
    }

    const auto & goal = input.back();
    if (output.empty() ||
      std::hypot(output.back().x - goal.x, output.back().y - goal.y) > 1e-4)
    {
      output.push_back(goal);
    }

    return output;
  }

  nav_msgs::msg::Path buildPathFromWorldPoints(
    const std::vector<geometry_msgs::msg::Point> & world_points,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    path.poses.reserve(world_points.size());

    for (std::size_t i = 0; i < world_points.size(); ++i) {
      const auto & point = world_points[i];

      double yaw = yawFromQuaternion(goal_pose.pose.orientation);
      if (i + 1 < world_points.size()) {
        const auto & next = world_points[i + 1];
        yaw = std::atan2(next.y - point.y, next.x - point.x);
      }

      path.poses.push_back(makePose(global_frame_, path.header.stamp, point.x, point.y, yaw));
    }

    if (!path.poses.empty()) {
      path.poses.front().pose.position = start_pose.pose.position;
      path.poses.back().pose.orientation = goal_pose.pose.orientation;
      path.poses.back().pose.position = goal_pose.pose.position;
    }
    return path;
  }

  nav_msgs::msg::Path buildPath(
    const nav_msgs::msg::OccupancyGrid & grid, const std::vector<int> & indices,
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) const
  {
    return buildPathFromWorldPoints(
      resampleWorldPoints(buildWorldPoints(grid, indices, start_pose, goal_pose)),
      start_pose, goal_pose);
  }

  // 轴线表的缓存。重建要逐格做「格号 → 世界坐标 → 语义格」，RMUC 尺寸下是十几万
  // 次；A* 每秒跑 5 次，每次重建会直接吃掉规划预算。
  //
  // 几何（尺寸/分辨率/origin）不变就能复用：表按格号索引，代价值变了不影响格号到
  // 语义格的映射关系。语义地图换了则由订阅回调清空。
  const TunnelAxisGrid & tunnelAxisFor(const nav_msgs::msg::OccupancyGrid & grid)
  {
    const bool geometry_changed =
      axis_grid_width_ != grid.info.width || axis_grid_height_ != grid.info.height ||
      axis_grid_resolution_ != grid.info.resolution ||
      axis_grid_origin_x_ != grid.info.origin.position.x ||
      axis_grid_origin_y_ != grid.info.origin.position.y;

    // 用单独的标志位而不是 tunnel_axis_.empty() 判「建过没」：没有隧道的地图建出来
    // 的表本来就是空的，拿 empty() 当判据会让最常见的情形每次规划都重扫一遍全图。
    if (!axis_grid_built_ || geometry_changed) {
      tunnel_axis_ = TunnelAxisGrid::build(grid, receiver_.map());
      axis_grid_built_ = true;
      axis_grid_width_ = grid.info.width;
      axis_grid_height_ = grid.info.height;
      axis_grid_resolution_ = grid.info.resolution;
      axis_grid_origin_x_ = grid.info.origin.position.x;
      axis_grid_origin_y_ = grid.info.origin.position.y;
    }
    return tunnel_axis_;
  }

  nav_msgs::msg::OccupancyGrid preparePlanningGrid(
    const nav_msgs::msg::OccupancyGrid & source_map) const
  {
    nav_msgs::msg::OccupancyGrid grid = source_map;
    if (grid.header.frame_id.empty()) {
      grid.header.frame_id = global_frame_;
    }
    if (apply_internal_inflation_) {
      applyInflationCostGradient(
        grid, inflation_radius_, obstacle_threshold_, inflation_cost_scaling_factor_);
    }
    return grid;
  }

  // A* 工作缓冲：按需扩到地图尺寸并重置（尺寸不变时零分配）。
  void resetAstarBuffers(std::size_t cells)
  {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    if (astar_g_score_.size() != cells) {
      astar_g_score_.assign(cells, kInf);
    } else {
      std::fill(astar_g_score_.begin(), astar_g_score_.end(), kInf);
    }
    if (astar_parent_.size() != cells) {
      astar_parent_.assign(cells, -1);
    } else {
      std::fill(astar_parent_.begin(), astar_parent_.end(), -1);
    }
    if (astar_closed_.size() != cells) {
      astar_closed_.assign(cells, 0);
    } else {
      std::fill(astar_closed_.begin(), astar_closed_.end(), 0);
    }
  }

  std::optional<nav_msgs::msg::Path> planPath(
    const nav_msgs::msg::OccupancyGrid & source_map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::OccupancyGrid grid = preparePlanningGrid(source_map);

    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    if (!worldToMap(grid, start.pose.position.x, start.pose.position.y, start_x, start_y)) {
      RCLCPP_ERROR(
        get_logger(), "Start pose (%.2f, %.2f) is outside the map",
        start.pose.position.x, start.pose.position.y);
      return std::nullopt;
    }
    if (!worldToMap(grid, goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      RCLCPP_ERROR(
        get_logger(), "Goal pose (%.2f, %.2f) is outside the map",
        goal.pose.position.x, goal.pose.position.y);
      return std::nullopt;
    }

    const int tolerance_cells =
      std::max(1, static_cast<int>(std::ceil(planner_tolerance_ / grid.info.resolution)));
    const auto start_index = nearestTraversableIndex(grid, start_x, start_y, tolerance_cells);
    const auto goal_index = nearestTraversableIndex(grid, goal_x, goal_y, tolerance_cells);
    if (!start_index || !goal_index) {
      RCLCPP_ERROR(
        get_logger(), "Cannot find traversable start/goal cells within %.2f m",
        planner_tolerance_);
      return std::nullopt;
    }

    const int total_cells = static_cast<int>(grid.data.size());
    // A* 工作缓冲提为成员复用：地图尺寸不变时零分配（否则每次规划要
    // 分配/释放 g_score 8B×cells + parent 4B×cells + closed 1B×cells）。
    // 本函数只在互斥回调组内执行，成员缓冲无并发风险。
    resetAstarBuffers(static_cast<std::size_t>(total_cells));
    auto & g_score = astar_g_score_;
    auto & parent = astar_parent_;
    auto & closed = astar_closed_;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    // 清障代价场用来让路径尽量远离障碍，而不是只追求最短。精确 EDT（O(n)，无堆）
    // 替代八邻域 Dijkstra，并按地图内容哈希缓存；惩罚值物化成 float 查表，
    // A* 扩展时零浮点开销。地图内容没变时（导航期常态）整场直接复用。
    if (use_clearance_cost_) {
      const std::uint64_t hash =
        clearanceFieldHash(grid, obstacle_threshold_, !allow_unknown_);
      if (!clearance_cache_.built || clearance_cache_.hash != hash) {
        if (clearance_cache_.cost.size() != grid.data.size()) {
          clearance_cache_.cost.assign(grid.data.size(), 0.0F);
        }
        std::vector<std::uint8_t> seeds(grid.data.size(), 0U);
        for (std::size_t i = 0; i < grid.data.size(); ++i) {
          seeds[i] =
            isOccupied(grid.data[i], obstacle_threshold_, !allow_unknown_) ? 1U : 0U;
        }
        const auto dist_cells = exactSquaredDistanceTransform(
          seeds, static_cast<int>(grid.info.width), static_cast<int>(grid.info.height));
        for (std::size_t i = 0; i < grid.data.size(); ++i) {
          const double dist_m = dist_cells[i] * grid.info.resolution;
          clearance_cache_.cost[i] = static_cast<float>(
            clearancePenalty(dist_m, clearance_desired_distance_, clearance_cost_weight_));
        }
        clearance_cache_.hash = hash;
        clearance_cache_.built = true;
      }
    }

    const auto heuristic = [&grid](int a, int b) {
      const int ax = a % static_cast<int>(grid.info.width);
      const int ay = a / static_cast<int>(grid.info.width);
      const int bx = b % static_cast<int>(grid.info.width);
      const int by = b / static_cast<int>(grid.info.width);
      return std::hypot(static_cast<double>(ax - bx), static_cast<double>(ay - by));
    };
    const auto clearance_cost = [this](int index) {
      if (!clearance_cache_.built) {
        return 0.0;
      }
      return static_cast<double>(clearance_cache_.cost[static_cast<std::size_t>(index)]);
    };

    // 轴线表按 grid 的格号索引，而 grid 来自 preparePlanningGrid（可能加了膨胀，但
    // 几何不变）。几何不变就能复用，所以只在语义地图或代价地图几何变化时重建。
    const TunnelAxisGrid & tunnel_axis = tunnelAxisFor(grid);

    g_score[static_cast<std::size_t>(*start_index)] = 0.0;
    open.push({*start_index, heuristic(*start_index, *goal_index)});

    // 8 邻域搜索：允许横移、竖移和斜移，路径更接近实际底盘可走轨迹。
    constexpr int directions[8][2] = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found = false;
    while (!open.empty()) {
      const auto current = open.top();
      open.pop();
      if (closed[static_cast<std::size_t>(current.index)] != 0U) {
        continue;
      }
      if (current.index == *goal_index) {
        found = true;
        break;
      }

      closed[static_cast<std::size_t>(current.index)] = 1;
      const int cx = current.index % static_cast<int>(grid.info.width);
      const int cy = current.index / static_cast<int>(grid.info.width);

      for (const auto & direction : directions) {
        const int nx = cx + direction[0];
        const int ny = cy + direction[1];
        if (!isTraversable(grid, nx, ny)) {
          continue;
        }
        if (direction[0] != 0 && direction[1] != 0) {
          if (!isTraversable(grid, cx + direction[0], cy) ||
            !isTraversable(grid, cx, cy + direction[1]))
          {
            continue;
          }
        }

        const int neighbor = static_cast<int>(gridIndex(grid, nx, ny));

        // 隧道内偏离轴线要加代价：洞只比车稍宽，斜着走一格就贴壁。纯几何代价图看不
        // 出这件事 —— 顶板和侧壁在点云里跟墙一样，语义地图是唯一的信息来源。
        //
        // 地图里没隧道时 tunnel_axis 是空的，stepAlignment 恒返回 1.0，这一项为 0。
        double axis_cost = 0.0;
        if (!tunnel_axis.empty() && tunnel_axis_cost_weight_ > 0.0) {
          const double alignment = tunnel_axis.stepAlignment(
            static_cast<std::size_t>(current.index), static_cast<std::size_t>(neighbor),
            direction[0], direction[1]);
          axis_cost = tunnel_axis_cost_weight_ * (1.0 - alignment);
        }

        const double step = (direction[0] != 0 && direction[1] != 0) ? std::sqrt(2.0) : 1.0;
        double turn_cost = 0.0;
        const int parent_index = parent[static_cast<std::size_t>(current.index)];
        if (parent_index >= 0) {
          const int px = parent_index % static_cast<int>(grid.info.width);
          const int py = parent_index / static_cast<int>(grid.info.width);
          const double prev_dx = static_cast<double>(cx - px);
          const double prev_dy = static_cast<double>(cy - py);
          const double next_dx = static_cast<double>(direction[0]);
          const double next_dy = static_cast<double>(direction[1]);
          const double prev_norm = std::hypot(prev_dx, prev_dy);
          const double next_norm = std::hypot(next_dx, next_dy);
          if (prev_norm > 1e-6 && next_norm > 1e-6) {
            const double cos_angle = std::clamp(
              (prev_dx * next_dx + prev_dy * next_dy) / (prev_norm * next_norm), -1.0, 1.0);
            turn_cost = turn_penalty_ * (1.0 - cos_angle);
          }
        }
        const double tentative =
          g_score[static_cast<std::size_t>(current.index)] + step * cellCost(grid, nx, ny) +
          turn_cost + axis_cost + clearance_cost(neighbor);
        if (tentative >= g_score[static_cast<std::size_t>(neighbor)]) {
          continue;
        }

        parent[static_cast<std::size_t>(neighbor)] = current.index;
        g_score[static_cast<std::size_t>(neighbor)] = tentative;
        open.push({neighbor, tentative + heuristic(neighbor, *goal_index)});
      }
    }

    if (!found) {
      RCLCPP_ERROR(get_logger(), "A* failed to find a path to the current goal");
      return std::nullopt;
    }

    auto indices = extractPathIndices(parent, *start_index, *goal_index);
    indices = prunePathIndices(grid, indices);
    return buildPath(grid, indices, start, goal);
  }

  bool pathTailTraversable(
    const nav_msgs::msg::OccupancyGrid & grid, const nav_msgs::msg::Path & path,
    std::size_t start_index) const
  {
    if (path.poses.empty()) {
      return false;
    }
    if (start_index + 1 >= path.poses.size()) {
      return true;
    }

    start_index = std::min(start_index, path.poses.size() - 1);
    for (std::size_t i = start_index; i + 1 < path.poses.size(); ++i) {
      int start_x = 0;
      int start_y = 0;
      int end_x = 0;
      int end_y = 0;
      const auto & start = path.poses[i].pose.position;
      const auto & end = path.poses[i + 1].pose.position;
      if (!worldToMap(grid, start.x, start.y, start_x, start_y) ||
        !worldToMap(grid, end.x, end.y, end_x, end_y))
      {
        return false;
      }
      if (!isSegmentTraversable(
          grid, static_cast<int>(gridIndex(grid, start_x, start_y)),
          static_cast<int>(gridIndex(grid, end_x, end_y))))
      {
        return false;
      }
    }
    return true;
  }

  std::optional<nav_msgs::msg::Path> tryStitchedReplan(
    const nav_msgs::msg::OccupancyGrid & source_map,
    const geometry_msgs::msg::PoseStamped & start)
  {
    if (!local_stitch_enabled_ || !last_path_ || last_path_->poses.size() < 3) {
      return std::nullopt;
    }

    const auto target = findStitchTarget(
      *last_path_, start.pose.position, stitch_min_lookahead_distance_, stitch_max_distance_);
    if (!target) {
      return std::nullopt;
    }

    const auto grid = preparePlanningGrid(source_map);
    if (!pathTailTraversable(grid, *last_path_, target->index)) {
      RCLCPP_DEBUG(get_logger(), "Previous path tail is no longer traversable; use full A*");
      return std::nullopt;
    }

    auto local_goal = target->pose;
    local_goal.header.frame_id = global_frame_;
    if (local_goal.header.stamp.sec == 0 && local_goal.header.stamp.nanosec == 0) {
      local_goal.header.stamp = now();
    }

    auto local_prefix = planPath(source_map, start, local_goal);
    if (!local_prefix || local_prefix->poses.size() < 2) {
      return std::nullopt;
    }

    auto stitched = stitchPath(*local_prefix, *last_path_, target->index);
    stitched.header.stamp = now();
    for (auto & pose : stitched.poses) {
      pose.header = stitched.header;
    }
    RCLCPP_INFO(
      get_logger(), "Local stitch replan connected to previous path index %zu (%.2f m ahead)",
      target->index, target->distance);
    return stitched;
  }

  void planFromCurrentPose(bool force_replan)
  {
    if (!map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No map received yet");
      return;
    }
    if (!goal_) {
      return;
    }

    // 规划失败冷却：如果上次规划失败且目标没有明显变化，等待冷却期过去。
    // 这防止对不可达目标以 planning_frequency 空转重试（洪水日志 + 白费算力）。
    // force_replan（来自用户重发目标、明确重规划请求）始终绕过冷却。
    if (!force_replan && plan_failure_cooldown_ > 0.0 && last_fail_time_ && last_fail_goal_) {
      const double since_fail = (now() - *last_fail_time_).seconds();
      if (since_fail < plan_failure_cooldown_) {
        const auto & prev = last_fail_goal_->pose.position;
        const double goal_shift = std::hypot(
          goal_->pose.position.x - prev.x, goal_->pose.position.y - prev.y);
        if (goal_shift < planner_tolerance_) {
          // 还在冷却，且目标没变，跳过。
          publishLastPath();
          return;
        }
      }
      // 冷却到期或目标移动了，清除失败记录，继续规划。
      last_fail_time_.reset();
      last_fail_goal_.reset();
    }

    geometry_msgs::msg::PoseStamped start;
    if (!getRobotPose(start)) {
      return;
    }

    if (!force_replan && last_planned_start_) {
      const double replan_distance = std::hypot(
        start.pose.position.x - last_planned_start_->pose.position.x,
        start.pose.position.y - last_planned_start_->pose.position.y);
      if (replan_distance < min_replan_distance_) {
        publishLastPath();
        return;
      }
    }

    auto goal = *goal_;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = global_frame_;
    }
    if (goal.header.frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Goal frame is %s, but this lightweight planner expects %s. Use RViz fixed frame map.",
        goal.header.frame_id.c_str(), global_frame_.c_str());
      return;
    }

    auto path = tryStitchedReplan(*map_, start);
    if (!path) {
      path = planPath(*map_, start, goal);
    }
    // A* 完成前先捕获代次快照，发布前再次确认目标没有改变。
    // 当节点运行在 component_container_mt 时，不同线程的回调（goal_sub、
    // replan_sub）可能在 A* 运行期间改变 plan_gen_；代次不匹配则丢弃。
    const uint64_t my_gen = plan_gen_.load(std::memory_order_acquire);
    if (!path || path->poses.empty()) {
      // 规划失败：记录时间和目标，进入冷却期。
      if (plan_failure_cooldown_ > 0.0) {
        last_fail_time_ = now();
        last_fail_goal_ = goal;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), static_cast<int>(plan_failure_cooldown_ * 1000),
          "Planning failed; retrying in %.1f s (goal: %.2f, %.2f)",
          plan_failure_cooldown_, goal.pose.position.x, goal.pose.position.y);
      }
      return;
    }

    // 路径验收：沿路径采样检查每个姿态是否落在障碍上。
    if (path_acceptance_enabled_) {
      bool path_ok = true;
      for (const auto & ps : path->poses) {
        int mx = 0;
        int my = 0;
        if (!worldToMap(*map_, ps.pose.position.x, ps.pose.position.y, mx, my)) {
          path_ok = false;
          break;
        }
        const auto cost = static_cast<int>(map_->data[gridIndex(*map_, mx, my)]);
        if (cost >= path_acceptance_max_cost_) {
          path_ok = false;
          break;
        }
      }
      if (!path_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Planned path rejected: passes through obstacle cost >= %d",
          path_acceptance_max_cost_);
        if (plan_failure_cooldown_ > 0.0) {
          last_fail_time_ = now();
          last_fail_goal_ = goal;
        }
        return;
      }
    }

    // 代次校验：A* 运行期间如果目标改变（component_container_mt 下并发回调），
    // plan_gen_ 已自增，此次结果不应发布。
    if (plan_gen_.load(std::memory_order_acquire) != my_gen) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Discarding stale plan (goal changed while A* was running, gen %lu → %lu)",
        my_gen, plan_gen_.load());
      return;
    }
    last_path_ = *path;
    path_pub_->publish(*path);
    last_planned_start_ = start;
    RCLCPP_DEBUG(
      get_logger(), "Published path with %zu poses on %s", path->poses.size(),
      path_topic_.c_str());
  }

  void publishLastPath()
  {
    if (!last_path_ || last_path_->poses.empty()) {
      return;
    }

    auto path = *last_path_;
    path.header.stamp = now();
    for (auto & pose : path.poses) {
      pose.header.stamp = path.header.stamp;
    }
    path_pub_->publish(path);
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string map_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string replan_request_topic_;
  double planning_frequency_{2.0};
  bool allow_unknown_{true};
  double unknown_cost_{1.4};
  int obstacle_threshold_{50};
  double inflation_radius_{0.25};
  bool apply_internal_inflation_{true};
  double map_cost_weight_{10.0};
  double map_cost_exponent_{2.0};
  double planner_tolerance_{0.5};
  bool use_clearance_cost_{true};
  double clearance_desired_distance_{0.6};
  double clearance_cost_weight_{3.0};
  bool replan_on_timer_{true};
  bool replan_on_source_update_{false};
  double min_replan_distance_{0.2};
  double turn_penalty_{0.2};
  bool local_stitch_enabled_{true};
  double stitch_min_lookahead_distance_{0.8};
  double stitch_max_distance_{3.0};
  bool path_prune_enabled_{true};
  int prune_segment_max_cost_{75};
  double path_resample_distance_{0.18};
  double inflation_cost_scaling_factor_{8.0};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::optional<geometry_msgs::msg::PoseStamped> last_planned_start_;
  std::optional<nav_msgs::msg::Path> last_path_;
  // 并发纪律：A* 同步跑在 executor 回调里（无后台线程）。component_container_mt
  // 下不同回调线程可能并发触发 planFromCurrentPose，靠 plan_gen_ 代次计数保证
  // 只有最新一次目标对应的结果能发布；last_path_ 等成员是「最后写入者胜」语义。
  std::atomic<uint64_t> plan_gen_{0};
  // 清障场缓存：按地图内容哈希，地图不变则整场复用（EDT 结果 + 物化惩罚表）。
  struct ClearanceCache
  {
    std::uint64_t hash{0};
    bool built{false};
    std::vector<float> cost;
  };
  ClearanceCache clearance_cache_;
  // A* 工作缓冲（成员复用，见 resetAstarBuffers）。
  std::vector<double> astar_g_score_;
  std::vector<int> astar_parent_;
  std::vector<uint8_t> astar_closed_;
  // 规划失败冷却：记录上次失败的时间和当时的目标，避免对不可达目标以规划
  // 频率空转重试。目标明显移动（超过 planner_tolerance_）时视为新目标，清除
  // 冷却计时。
  std::optional<rclcpp::Time> last_fail_time_;
  std::optional<geometry_msgs::msg::PoseStamped> last_fail_goal_;
  double plan_failure_cooldown_{2.0};
  // 路径发布前验收：拦截穿越障碍的路径（极端情况下拼接段可能走入新障碍）。
  bool path_acceptance_enabled_{true};
  int path_acceptance_max_cost_{85};

  // 语义地图与派生的轴线表。收不到语义地图时表是空的，A* 退回纯几何行为。
  std::string semantic_map_topic_;
  double tunnel_axis_cost_weight_{2.0};
  SemanticMapReceiver receiver_;
  TunnelAxisGrid tunnel_axis_;
  bool axis_grid_built_{false};
  unsigned int axis_grid_width_{0};
  unsigned int axis_grid_height_{0};
  float axis_grid_resolution_{0.0F};
  double axis_grid_origin_x_{0.0};
  double axis_grid_origin_y_{0.0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // 串行化订阅/定时器回调（mt 容器下共享成员 map_/goal_/last_path_ 等）。
  rclcpp::CallbackGroup::SharedPtr planner_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmGlobalPlanner)
