#include "distance_transform.hpp"
#include "grid_utils.hpp"
#include "semantic_map_consumer.hpp"
#include "shared_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace navigation2
{

// 逐格膨胀上限缓存：local cell → (是否在隧道影响区, 该格上限)。
// 局部栅格 origin 每帧随车按整数格平移（snapOrigin 保证 origin 是分辨率的整数倍），
// 语义格映射随之整体平移，只需平移缓存 + 补新露出的边，不必每帧逐格
// mapToWorld + 查语义格。语义地图/尺寸/分辨率变化时整体重建。
struct InflationLimitCache
{
  int width{0};
  int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  // in_tunnel[i] == 1 表示第 i 格落在隧道影响区内。
  std::vector<std::uint8_t> in_tunnel;
  // limit[i]：隧道格 = min(inflation_radius_, clearance)，非隧道格 = inflation_radius_。
  // 有隧道时直接作为返回值；无隧道时返回空向量（与 makeInflationRadiusLimit 一致）。
  std::vector<float> limit;
  bool has_tunnel{false};
};

// 每格「距上次命中的帧数」缓存，给 hit/miss 概率占据的 keep_time→clear_time 时间
// 衰减用（sentry 的 decay 语义，替代 previous_obstacle_decay 的固定整数衰减）。
//
// age=0 表示本帧刚命中；每过一帧 seed 时递增 1。命中（accumulateHit）时清零。
// 随栅格 origin 按整数格平移，方向与 seedFromPreviousGrid / shiftInflationLimitCache
// 一致：新格 (x,y) 复用旧格 (x+shift_x, y+shift_y)。
//
// 为什么用帧数而不是时间戳：update_frequency 固定（10Hz），帧数×周期即时间，省掉
// 每格存 rclcpp::Time（16 字节）——uint16 每格 2 字节，且衰减只在 seed 时算一次。
struct HitAgeCache
{
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::uint16_t> age;

  void reset(const nav_msgs::msg::OccupancyGrid & grid)
  {
    width = static_cast<int>(grid.info.width);
    height = static_cast<int>(grid.info.height);
    origin_x = grid.info.origin.position.x;
    origin_y = grid.info.origin.position.y;
    age.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
  }

  // 平移 + 全量递增。返回平移递增后的新数组（新露出的边为 0）。不直接改写自身，
  // 由调用方 swap，避免 seed 过程中新旧数组混用。
  std::vector<std::uint16_t> shiftAndIncrement(int shift_x, int shift_y) const
  {
    const std::size_t n = age.size();
    std::vector<std::uint16_t> next(n, 0);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int old_x = x + shift_x;
        const int old_y = y + shift_y;
        if (old_x < 0 || old_x >= width || old_y < 0 || old_y >= height) {
          continue;  // 新露出的边，age=0
        }
        const std::size_t old_idx =
          static_cast<std::size_t>(old_y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(old_x);
        const std::uint16_t a = age[old_idx];
        // 封顶防溢出：超过 clear_frames 后不再有意义，停在一个足够大的值即可。
        next[static_cast<std::size_t>(y) * width + x] =
          (a < 65535) ? static_cast<std::uint16_t>(a + 1) : a;
      }
    }
    return next;
  }

  void hit(int x, int y)
  {
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return;
    }
    age[static_cast<std::size_t>(y) * width + x] = 0;
  }
};

class RmLocalCostmap : public rclcpp::Node
{
public:
  explicit RmLocalCostmap(const rclcpp::NodeOptions & options)
  : Node("rm_local_costmap", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    // mt 容器下订阅回调与定时器回调读写同一批 latest_*/semantic 成员，
    // 全部放进一个互斥回调组串行化（与 rm_global_planner 同款纪律）。
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    // 按开关订阅不同传感器输入。
    if (subscribe_scan_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          latest_scan_ = std::move(msg);
        },
        sub_options);
    }
    if (subscribe_pointcloud_) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          latest_pointcloud_ = std::move(msg);
        },
        sub_options);
    }

    // 语义地图：局部代价地图自己不加载 /map，但隧道顶板必须在这里也被滤掉 ——
    // 局部图才是 MPC 的碰撞依据，只在全局图上放行等于让车看见洞口却撞在顶板上。
    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](decision_interfaces::msg::SemanticMap::ConstSharedPtr msg) {
        try {
          // 内容没变就不重建 —— rm_map_server 每秒重发一次，重建 RMUC 大小的方向场
          // 要 16 万次三角函数，而这个节点和 MPC 共用执行器。
          if (!receiver_.update(*msg)) {
            return;
          }
        } catch (const std::exception & ex) {
          // 拒收整帧，保留上一张好图。半张图会让隧道格错位到别的位置。
          RCLCPP_ERROR(get_logger(), "Rejected semantic map: %s", ex.what());
          return;
        }
        // 隧道影响区（本体 + 边距）随地图重建。顶板豁免和膨胀上限都按它判定：
        // 门楣点云和洞口格都在本体格外一到两格，只认本体格会把洞口整体封死。
        tunnel_region_ = TunnelRegionGrid::build(receiver_.map(), tunnel_margin_m_);
        // 影响区变了，逐格膨胀上限缓存失效，下次 updateAndPublish 整体重建。
        infl_limit_cache_ = InflationLimitCache{};
        RCLCPP_INFO(
          get_logger(), "Local costmap got semantic map: %zu tunnels",
          receiver_.map().tunnels().size());
      },
      sub_options);

    // 输出栅格给规划器和控制器。
    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, costmap_qos);
    raw_costmap_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(raw_costmap_topic_, costmap_qos);
    footprint_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(footprint_topic_, rclcpp::QoS(1));
    marked_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(marked_cloud_topic_, rclcpp::QoS(1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_frequency_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        updateAndPublish();
      },
      cb_group_);

    RCLCPP_INFO(
      get_logger(), "rm_local_costmap ready: scan=%s pointcloud=%s output=%s",
      subscribe_scan_ ? scan_topic_.c_str() : "disabled",
      subscribe_pointcloud_ ? pointcloud_topic_.c_str() : "disabled", costmap_topic_.c_str());
  }

private:
  void loadParameters()
  {
    // 局部代价地图尺寸和传感器开关。
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/segmentation/obstacle");
    semantic_map_topic_ =
      declare_parameter<std::string>("semantic_map_topic", "/map_server/semantic_map");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap");
    raw_costmap_topic_ =
      declare_parameter<std::string>("raw_costmap_topic", "/local_costmap/costmap_raw");
    footprint_topic_ =
      declare_parameter<std::string>("footprint_topic", "/local_costmap/published_footprint");
    marked_cloud_topic_ =
      declare_parameter<std::string>("marked_cloud_topic", "/local_costmap/voxel_marked_cloud");
    subscribe_scan_ = declare_parameter<bool>("subscribe_scan", true);
    subscribe_pointcloud_ = declare_parameter<bool>("subscribe_pointcloud", false);
    update_frequency_ = declare_parameter<double>("update_frequency", 20.0);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 10.0);
    width_m_ = declare_parameter<double>("width", 5.0);
    height_m_ = declare_parameter<double>("height", 5.0);
    resolution_ = declare_parameter<double>("resolution", 0.05);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.35);
    inflation_cost_scaling_factor_ =
      declare_parameter<double>("inflation_cost_scaling_factor", 15.0);
    raytrace_max_range_ = declare_parameter<double>("raytrace_max_range", 6.0);
    obstacle_min_range_ = declare_parameter<double>("obstacle_min_range", 0.1);
    obstacle_max_range_ = declare_parameter<double>("obstacle_max_range", 6.0);
    // 障碍高度带，相对机器人底盘（base_link）而非固定的 map z=0 平面。
    // 名字里带 _to_robo 是为了不再被误读成「离地高度」：Super-LIO 的 odom 原点
    // 锚在开机瞬间的雷达位姿上，所以 map 系的 z=0 是雷达平面，地面在
    // -(雷达安装高度) 处。拿绝对 z 比常数必然错。参考 rose_navigation 的
    // bottom_z_to_robo_z / top_z_to_robo_z。
    obstacle_z_min_to_robo_ = declare_parameter<double>("obstacle_z_min_to_robo", 0.05);
    obstacle_z_max_to_robo_ = declare_parameter<double>("obstacle_z_max_to_robo", 2.0);
    // 隧道影响区边距：本体格向外扩这么多米。区内点云一律不标记（见
    // processPointCloud），膨胀上限也压到通道余量。门楣/顶板前沿的点云和洞口
    // 正前方的格子都落在本体格外一到两格，0 边距时这排点被标成致命格横在洞口
    // 上、洞口格又吃到两侧墙的全量膨胀，洞口整体被封死。语义与
    // global_costmap_node.cpp 的同名参数一致，两边必须一起改。
    tunnel_margin_m_ = declare_parameter<double>("tunnel_margin_m", 0.20);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);
    observation_timeout_ = declare_parameter<double>("observation_timeout", 0.5);
    snap_origin_to_grid_ = declare_parameter<bool>("snap_origin_to_grid", true);
    use_observation_stamp_for_origin_ =
      declare_parameter<bool>("use_observation_stamp_for_origin", false);
    use_latest_sensor_transform_ =
      declare_parameter<bool>("use_latest_sensor_transform", false);
    fallback_to_latest_sensor_transform_ =
      declare_parameter<bool>("fallback_to_latest_sensor_transform", false);
    update_on_new_observation_only_ =
      declare_parameter<bool>("update_on_new_observation_only", false);
    reuse_previous_grid_ = declare_parameter<bool>("reuse_previous_grid", true);
    previous_obstacle_decay_ = declare_parameter<int>("previous_obstacle_decay", 0);
    // 命中增量：点云/扫描命中某格时，代价累加 hit_inc 而不是直接置 100（sentry 的
    // hit/miss 概率占据在 Lucifer 2D 栅格上的简化等价物）。默认 100 = 单帧即致命
    // （保持历史行为）；调小（如 50）需要多帧确认才达到 obstacle_threshold，抑制
    // 瞬时噪声/孤立误检。与 previous_obstacle_decay 的「每帧衰减」构成 hit/miss 闭环。
    hit_inc_ = declare_parameter<int>("hit_inc", 100);
    // 动态残影时间衰减（sentry 的 keep_time→clear_time 语义，替代整数衰减）：
    // 命中后 age < keep_time 保持致命；keep_time≤age<clear_time 线性衰减到 free；
    // age≥clear_time 清除。enable=false 时退回 previous_obstacle_decay 整数衰减。
    dynamic_decay_enable_ = declare_parameter<bool>("dynamic_decay.enable", false);
    dynamic_decay_keep_time_ = declare_parameter<double>("dynamic_decay.keep_time", 0.8);
    dynamic_decay_clear_time_ = declare_parameter<double>("dynamic_decay.clear_time", 1.2);
    // 保守先验合并（sentry 的「先验 occupied 强制并入、先验 free 不抹在线障碍」）：
    // 语义地图的 OBSTACLE 格强制标进局部代价地图。收不到语义地图时静默失效。
    prior_merge_enable_ = declare_parameter<bool>("prior_merge.enable", false);
    // 进程内距离场开关：给 MINCO 障碍代价与 ESDF 梯度脱困共用（sentry 的
    // Signed ESDF 在 Lucifer 2D 栅格上的落地）。默认关 —— 只有真正需要 soft 障碍
    // 代价 / 梯度脱困时才开，避免每次发布多一次 O(n) 距离变换。
    distance_field_enable_ = declare_parameter<bool>("distance_field_enable", false);
  }

  bool stampIsZero(const builtin_interfaces::msg::Time & stamp) const
  {
    return stamp.sec == 0 && stamp.nanosec == 0;
  }

  tf2::TimePoint headerTime(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stampIsZero(stamp)) {
      return tf2::TimePointZero;
    }
    return tf2_ros::fromMsg(stamp);
  }

  bool sameStamp(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs) const
  {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
  }

  bool stampNewer(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs) const
  {
    if (lhs.sec != rhs.sec) {
      return lhs.sec > rhs.sec;
    }
    return lhs.nanosec > rhs.nanosec;
  }

  rclcpp::Time stampToRosTime(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stampIsZero(stamp)) {
      return now();
    }
    return rclcpp::Time(stamp, get_clock()->get_clock_type());
  }

  bool getRobotTransform(
    const tf2::TimePoint & stamp, geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_->lookupTransform(
        global_frame_, robot_base_frame_, stamp, tf2::durationFromSec(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot get robot transform %s -> %s: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool lookupSensorTransform(
    const std::string & sensor_frame,
    const builtin_interfaces::msg::Time & sensor_stamp,
    const tf2::TimePoint & reference_time,
    geometry_msgs::msg::TransformStamped & transform)
  {
    // 优先按传感器时间查 TF。
    const auto transform_time =
      use_latest_sensor_transform_ ? reference_time : headerTime(sensor_stamp);

    try {
      transform = tf_buffer_->lookupTransform(
        global_frame_, sensor_frame, transform_time, tf2::durationFromSec(transform_tolerance_));
      return true;
    } catch (const tf2::TransformException & ex) {
      if (!use_latest_sensor_transform_ && fallback_to_latest_sensor_transform_) {
        try {
          transform = tf_buffer_->lookupTransform(
            global_frame_, sensor_frame, reference_time,
            tf2::durationFromSec(transform_tolerance_));
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Cannot transform %s to %s at sensor stamp, using latest TF fallback: %s",
            sensor_frame.c_str(), global_frame_.c_str(), ex.what());
          return true;
        } catch (const tf2::TransformException & fallback_ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot transform %s to %s using sensor stamp or latest TF: %s",
            sensor_frame.c_str(), global_frame_.c_str(), fallback_ex.what());
          return false;
        }
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot transform %s to %s: %s",
        sensor_frame.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool observationIsFresh(const builtin_interfaces::msg::Time & stamp) const
  {
    // 超时的观测直接丢掉。
    if (observation_timeout_ <= 0.0 || stampIsZero(stamp)) {
      return true;
    }

    const auto age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    // 未来时间戳同样拒绝：负 age 不是“更新鲜”，而是传感器/仿真时钟契约错误。
    return std::isfinite(age) && age >= 0.0 && age <= observation_timeout_;
  }

  bool hasFreshObservation()
  {
    bool have_observation = false;

    if (latest_scan_) {
      if (!observationIsFresh(latest_scan_->header.stamp)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Latest scan is stale, ignore it");
      } else {
        have_observation = true;
      }
    }

    if (latest_pointcloud_) {
      if (!observationIsFresh(latest_pointcloud_->header.stamp)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Latest pointcloud is stale, ignore it");
      } else {
        have_observation = true;
      }
    }

    return have_observation;
  }

  bool latestObservationStamp(builtin_interfaces::msg::Time & stamp) const
  {
    bool found = false;

    if (latest_scan_ && observationIsFresh(latest_scan_->header.stamp)) {
      stamp = latest_scan_->header.stamp;
      found = true;
    }

    if (latest_pointcloud_ && observationIsFresh(latest_pointcloud_->header.stamp)) {
      if (!found || stampIsZero(stamp) ||
        (!stampIsZero(latest_pointcloud_->header.stamp) &&
        stampNewer(latest_pointcloud_->header.stamp, stamp)))
      {
        stamp = latest_pointcloud_->header.stamp;
      }
      found = true;
    }

    return found;
  }

  double snapOrigin(double value) const
  {
    if (!snap_origin_to_grid_ || resolution_ <= 0.0) {
      return value;
    }
    return std::floor(value / resolution_) * resolution_;
  }

  nav_msgs::msg::OccupancyGrid makeEmptyGrid(
    const geometry_msgs::msg::TransformStamped & robot_transform,
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = global_frame_;
    grid.header.stamp = stamp;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<unsigned int>(std::ceil(width_m_ / resolution_));
    grid.info.height = static_cast<unsigned int>(std::ceil(height_m_ / resolution_));
    const double grid_width = static_cast<double>(grid.info.width) * resolution_;
    const double grid_height = static_cast<double>(grid.info.height) * resolution_;
    grid.info.origin.position.x =
      snapOrigin(robot_transform.transform.translation.x - grid_width * 0.5);
    grid.info.origin.position.y =
      snapOrigin(robot_transform.transform.translation.y - grid_height * 0.5);
    grid.info.origin.orientation = quaternionFromYaw(0.0);
    grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);
    return grid;
  }

  bool worldToMapContinuous(
    const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y,
    double & map_x, double & map_y) const
  {
    if (grid.info.resolution <= 0.0F) {
      return false;
    }

    const double origin_x = grid.info.origin.position.x;
    const double origin_y = grid.info.origin.position.y;
    const double yaw = yawFromQuaternion(grid.info.origin.orientation);
    const double dx = world_x - origin_x;
    const double dy = world_y - origin_y;
    const double cos_yaw = std::cos(-yaw);
    const double sin_yaw = std::sin(-yaw);
    const double local_x = dx * cos_yaw - dy * sin_yaw;
    const double local_y = dx * sin_yaw + dy * cos_yaw;
    map_x = local_x / grid.info.resolution;
    map_y = local_y / grid.info.resolution;
    return std::isfinite(map_x) && std::isfinite(map_y);
  }

  bool clippedEndpointCell(
    const nav_msgs::msg::OccupancyGrid & grid, int origin_x, int origin_y,
    double endpoint_x, double endpoint_y, int & end_x, int & end_y) const
  {
    if (worldToMap(grid, endpoint_x, endpoint_y, end_x, end_y)) {
      return true;
    }

    double continuous_x = 0.0;
    double continuous_y = 0.0;
    if (!worldToMapContinuous(grid, endpoint_x, endpoint_y, continuous_x, continuous_y)) {
      return false;
    }

    const double start_x = static_cast<double>(origin_x) + 0.5;
    const double start_y = static_cast<double>(origin_y) + 0.5;
    const double dx = continuous_x - start_x;
    const double dy = continuous_y - start_y;
    if (std::hypot(dx, dy) <= 1e-6) {
      return false;
    }

    double t = 1.0;
    const double max_x = static_cast<double>(grid.info.width) - 1e-3;
    const double max_y = static_cast<double>(grid.info.height) - 1e-3;
    if (dx > 0.0) {
      t = std::min(t, (max_x - start_x) / dx);
    } else if (dx < 0.0) {
      t = std::min(t, (0.0 - start_x) / dx);
    }
    if (dy > 0.0) {
      t = std::min(t, (max_y - start_y) / dy);
    } else if (dy < 0.0) {
      t = std::min(t, (0.0 - start_y) / dy);
    }

    if (!std::isfinite(t) || t <= 0.0) {
      return false;
    }

    end_x = std::clamp(
      static_cast<int>(std::floor(start_x + dx * t)), 0,
      static_cast<int>(grid.info.width) - 1);
    end_y = std::clamp(
      static_cast<int>(std::floor(start_y + dy * t)), 0,
      static_cast<int>(grid.info.height) - 1);
    return inBounds(grid, end_x, end_y);
  }

  void clearRaytraceLine(
    nav_msgs::msg::OccupancyGrid & grid, int x0, int y0, int x1, int y1) const
  {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;

    while (true) {
      if (inBounds(grid, x, y)) {
        grid.data[gridIndex(grid, x, y)] = 0;
      }
      if (x == x1 && y == y1) {
        break;
      }
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
  }

  bool shouldPublish(const rclcpp::Time & stamp) const
  {
    if (publish_frequency_ <= 0.0) {
      return false;
    }
    if (last_publish_time_.nanoseconds() == 0) {
      return true;
    }
    const double elapsed = (stamp - last_publish_time_).seconds();
    return elapsed + 1e-6 >= 1.0 / std::max(0.1, publish_frequency_);
  }


  void publishCurrentFootprint()
  {
    if (footprint_pub_->get_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    if (!getRobotTransform(tf2::TimePointZero, robot_transform)) {
      return;
    }

    publishFootprint(robot_transform, now());
  }

  bool gridCompatible(
    const nav_msgs::msg::OccupancyGrid & lhs,
    const nav_msgs::msg::OccupancyGrid & rhs) const
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width &&
           lhs.info.height == rhs.info.height &&
           std::abs(lhs.info.resolution - rhs.info.resolution) <= 1e-6 &&
           std::abs(yawFromQuaternion(lhs.info.origin.orientation) -
             yawFromQuaternion(rhs.info.origin.orientation)) <= 1e-6;
  }

  void seedFromPreviousGrid(nav_msgs::msg::OccupancyGrid & grid)
  {
    if (!reuse_previous_grid_ || !previous_raw_grid_ ||
      !gridCompatible(*previous_raw_grid_, grid))
    {
      // 无上一帧可复用：帧龄缓存整拍重建（新露出的所有格 age=0）。
      hit_age_.reset(grid);
      return;
    }

    const auto & previous = *previous_raw_grid_;
    const double dx =
      (grid.info.origin.position.x - previous.info.origin.position.x) / resolution_;
    const double dy =
      (grid.info.origin.position.y - previous.info.origin.position.y) / resolution_;
    const int offset_x = static_cast<int>(std::lround(dx));
    const int offset_y = static_cast<int>(std::lround(dy));
    if (std::abs(dx - static_cast<double>(offset_x)) > 1e-3 ||
      std::abs(dy - static_cast<double>(offset_y)) > 1e-3)
    {
      hit_age_.reset(grid);
      return;
    }

    // 帧龄先平移 + 递增：上一帧命中过的格 age 前进一格，供下面按 keep/clear 衰减。
    // 用「上一帧的 age」而不是「本帧」是因为本帧的命中要等 processPointCloud 才发生。
    auto next_age = hit_age_.shiftAndIncrement(offset_x, offset_y);

    if (dynamic_decay_enable_) {
      // sentry 的 keep_time→clear_time 时间衰减：age<keep 保持致命；
      // keep≤age<clear 线性衰减到 free；age≥clear 清除。
      const double period = 1.0 / std::max(1e-3, update_frequency_);
      const int keep_frames = std::max(1, static_cast<int>(std::ceil(
        std::max(0.0, dynamic_decay_keep_time_) / period)));
      const int clear_frames = std::max(keep_frames + 1, static_cast<int>(std::ceil(
        std::max(0.0, dynamic_decay_clear_time_) / period)));
      for (unsigned int y = 0; y < grid.info.height; ++y) {
        const int previous_y = static_cast<int>(y) + offset_y;
        if (previous_y < 0 || previous_y >= static_cast<int>(previous.info.height)) {
          continue;
        }
        for (unsigned int x = 0; x < grid.info.width; ++x) {
          const int previous_x = static_cast<int>(x) + offset_x;
          if (previous_x < 0 || previous_x >= static_cast<int>(previous.info.width)) {
            continue;
          }
          const auto previous_index = gridIndex(previous, previous_x, previous_y);
          if (previous.data[previous_index] <= 0) {
            continue;
          }
          const std::size_t idx =
            static_cast<std::size_t>(y) * grid.info.width + static_cast<std::size_t>(x);
          const int age = next_age[idx];
          int cost = 0;
          if (age < keep_frames) {
            cost = 100;
          } else if (age < clear_frames) {
            cost = static_cast<int>(std::lround(
              100.0 * static_cast<double>(clear_frames - age) /
              static_cast<double>(clear_frames - keep_frames)));
          }
          if (cost > 0) {
            const auto current_index = gridIndex(grid, static_cast<int>(x), static_cast<int>(y));
            grid.data[current_index] = std::max(
              grid.data[current_index], static_cast<int8_t>(cost));
          }
        }
      }
    } else {
      // 历史行为：固定整数衰减，与帧龄无关。
      for (unsigned int y = 0; y < grid.info.height; ++y) {
        const int previous_y = static_cast<int>(y) + offset_y;
        if (previous_y < 0 || previous_y >= static_cast<int>(previous.info.height)) {
          continue;
        }
        for (unsigned int x = 0; x < grid.info.width; ++x) {
          const int previous_x = static_cast<int>(x) + offset_x;
          if (previous_x < 0 || previous_x >= static_cast<int>(previous.info.width)) {
            continue;
          }
          const auto previous_index = gridIndex(previous, previous_x, previous_y);
          int persisted_cost = static_cast<int>(previous.data[previous_index]);
          if (persisted_cost <= 0) {
            continue;
          }
          persisted_cost = std::max(0, persisted_cost - previous_obstacle_decay_);
          if (persisted_cost <= 0) {
            continue;
          }
          const auto current_index = gridIndex(grid, static_cast<int>(x), static_cast<int>(y));
          grid.data[current_index] = std::max(
            grid.data[current_index], static_cast<int8_t>(persisted_cost));
        }
      }
    }

    hit_age_.age.swap(next_age);
    hit_age_.origin_x = grid.info.origin.position.x;
    hit_age_.origin_y = grid.info.origin.position.y;
  }

  // 命中累积：代价累加 hit_inc 而不是直接置 100。默认 hit_inc=100 时等价于历史
  // 二值行为（单帧即致命）；调小后需要多帧确认才达到 obstacle_threshold，配合
  // 时间衰减（dynamic_decay）或整数衰减（previous_obstacle_decay）构成 hit/miss
  // 概率闭环（sentry 概率占据的 2D 简化）。命中同时把帧龄清零。int8 数据区天然夹
  // 在 [0,100]，min/max 防溢出。
  void accumulateHit(nav_msgs::msg::OccupancyGrid & grid, int x, int y)
  {
    if (!inBounds(grid, x, y)) {
      return;
    }
    const std::size_t idx = gridIndex(grid, x, y);
    // unknown(-1) 格归零再累加，保证 hit_inc=100 时与历史「直接置 100」完全等价
    // （unknown 命中后仍是 100，而不是 -1+100=99）。
    const int cur = std::max(0, static_cast<int>(grid.data[idx]));
    const int next = cur + hit_inc_;
    grid.data[idx] = static_cast<int8_t>(std::clamp(next, 0, 100));
    hit_age_.hit(x, y);
  }

  void markRobotFootprintFree(
    nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::TransformStamped & robot_transform)
  {
    int center_mx = 0;
    int center_my = 0;
    if (!worldToMap(
        grid, robot_transform.transform.translation.x, robot_transform.transform.translation.y,
        center_mx, center_my))
    {
      return;
    }
    const int radius_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int x = center_mx + dx;
        const int y = center_my + dy;
        if (inBounds(grid, x, y)) {
          grid.data[gridIndex(grid, x, y)] = 0;
          hit_age_.hit(x, y);  // 脚下清空的同时帧龄归零，不再参与时间衰减
        }
      }
    }
  }

  // 保守先验合并：语义地图的 OBSTACLE 格强制标进局部代价地图（先验障碍只加不减）。
  // sentry 的契约「先验 occupied 强制并入、先验 free 绝不抹掉在线障碍」在 Lucifer 的
  // 落地：只把先验 OBSTACLE 标成致命（=100），先验 free/隧道/未知格不做任何操作，
  // 因此不会清掉 processPointCloud 已经标上的在线障碍。
  //
  // 为什么局部图需要先验障碍：它现在只吃 /segmentation/obstacle（在线点云）。先验
  // 地图里标注的静态障碍若被点云漏扫（遮挡、反射差、车体自挡），局部层就看不见，
  // MPC 会贴着它走。合并后与全局图/语义一致。
  void mergePriorObstacles(nav_msgs::msg::OccupancyGrid & grid)
  {
    if (!prior_merge_enable_ || !receiver_.map().valid()) {
      return;
    }
    const auto & map = receiver_.map();
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        double world_x = 0.0;
        double world_y = 0.0;
        mapToWorld(grid, x, y, world_x, world_y);
        const auto cell = map.geometry().containingCell(Eigen::Vector2d(world_x, world_y));
        if (!cell) {
          continue;
        }
        if (map.terrainAtCell(cell->x(), cell->y()) ==
          static_cast<std::uint8_t>(TerrainType::OBSTACLE))
        {
          grid.data[gridIndex(grid, x, y)] = 100;
        }
      }
    }
  }

  void processScan(
    nav_msgs::msg::OccupancyGrid & grid,
    std::vector<geometry_msgs::msg::Point> & marked_points,
    const tf2::TimePoint & reference_time,
    const bool collect_marked_points)
  {
    if (!latest_scan_) {
      return;
    }

    const auto & scan = *latest_scan_;
    if (!observationIsFresh(scan.header.stamp)) {
      return;
    }

    const std::string scan_frame =
      scan.header.frame_id.empty() ? robot_base_frame_ : scan.header.frame_id;

    geometry_msgs::msg::TransformStamped scan_to_global;
    if (!lookupSensorTransform(scan_frame, scan.header.stamp, reference_time, scan_to_global)) {
      return;
    }

    const auto scan_origin = transformPoint(scan_to_global, 0.0, 0.0, 0.0);
    int origin_x = 0;
    int origin_y = 0;
    if (!worldToMap(grid, scan_origin.x, scan_origin.y, origin_x, origin_y)) {
      return;
    }

    std::vector<GridCell> endpoints_to_mark;
    endpoints_to_mark.reserve(scan.ranges.size());

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float raw_range = scan.ranges[i];
      const bool finite = std::isfinite(raw_range);
      if (!finite && !std::isinf(raw_range)) {
        continue;
      }

      const double usable_range = finite ?
        std::clamp<double>(raw_range, 0.0, raytrace_max_range_) :
        raytrace_max_range_;
      if (usable_range < obstacle_min_range_) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double local_x = usable_range * std::cos(angle);
      const double local_y = usable_range * std::sin(angle);
      const auto endpoint = transformPoint(scan_to_global, local_x, local_y, 0.0);

      int end_x = 0;
      int end_y = 0;
      const bool endpoint_in_grid =
        worldToMap(grid, endpoint.x, endpoint.y, end_x, end_y);
      if (!endpoint_in_grid &&
        !clippedEndpointCell(grid, origin_x, origin_y, endpoint.x, endpoint.y, end_x, end_y))
      {
        continue;
      }

      clearRaytraceLine(grid, origin_x, origin_y, end_x, end_y);

      if (endpoint_in_grid && finite && raw_range >= obstacle_min_range_ &&
        raw_range <= obstacle_max_range_)
      {
        endpoints_to_mark.push_back({end_x, end_y});
        if (collect_marked_points) {
          marked_points.push_back(endpoint);
        }
      }
    }

    for (const auto & endpoint : endpoints_to_mark) {
      if (inBounds(grid, endpoint.x, endpoint.y)) {
        accumulateHit(grid, endpoint.x, endpoint.y);
      }
    }
  }

  // 该点是否落在隧道影响区（本体 + 边距）内。区内点云一律不标记：顶板、门楣、
  // 侧壁上沿在点云里跟墙没有区别，任何高度阈值都在「滤掉结构」和「漏掉真障碍」
  // 之间赌 —— 而净高够不够、姿态收没收是电控的职责，导航只负责把车沿轴线送进
  // 洞。能不能进完全由静态地图的壁面致命格决定。
  //
  // 影响区内的放行判据由 TunnelRegionGrid::pointOutsideCorridor 统一给出
  // （PCA 轴走廊：走廊内先验放行、走廊外恢复多帧确认），局部/全局两层共用。

  void processPointCloud(
    nav_msgs::msg::OccupancyGrid & grid,
    std::vector<geometry_msgs::msg::Point> & marked_points,
    const tf2::TimePoint & reference_time,
    const bool collect_marked_points,
    const double robo_z)
  {
    if (!latest_pointcloud_) {
      return;
    }

    const auto & cloud = *latest_pointcloud_;
    if (!observationIsFresh(cloud.header.stamp)) {
      return;
    }

    const std::string cloud_frame =
      cloud.header.frame_id.empty() ? robot_base_frame_ : cloud.header.frame_id;

    geometry_msgs::msg::TransformStamped cloud_to_global;
    if (!lookupSensorTransform(cloud_frame, cloud.header.stamp, reference_time, cloud_to_global)) {
      return;
    }

    const navigation2::PlanarFrame cloud_frame_ = navigation2::makePlanarFrame(cloud_to_global);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        const double range = std::hypot(static_cast<double>(*iter_x), static_cast<double>(*iter_y));
        if (range < obstacle_min_range_ || range > obstacle_max_range_) {
          continue;
        }

        const auto point = applyPlanarFrame(cloud_frame_, *iter_x, *iter_y, *iter_z);
        // 高度判定必须是「相对机器人底盘」的差值，两种绝对判法都会让 20cm 高台
        // 从代价地图上静默消失，没有任何报错：
        // 1) 在 transformPoint 之前拿点云原始 z 比。点云 frame 是 livox_frame，
        //    原点在雷达上，台面在那个系里只有 +0.025，被 0.05 的阈值滤掉。
        // 2) 变换到 map 之后跟固定的 z=0 比。Super-LIO 的 odom 原点锚在开机瞬间
        //    的雷达位姿上（tf_pose_utils.hpp: lidar_odom * livox_to_base），所以
        //    map 系的 z=0 是雷达平面而不是地面 —— 实测 RMUL.pcd 的地面主峰在
        //    -0.17，正好是 -(雷达安装高度 0.175)。台面绝对 z 只有 +0.025。
        // 做差之后，任何固定的雷达平面偏移都被抵消掉，也不受定位 z 漂移影响。
        const double height_to_robo = point.z - robo_z;
        if (height_to_robo < obstacle_z_min_to_robo_ ||
          height_to_robo > obstacle_z_max_to_robo_)
        {
          continue;
        }
        const bool in_tunnel_region =
          tunnel_region_.specNearPoint(point.x, point.y) != nullptr;
        if (in_tunnel_region && !tunnel_region_.pointOutsideCorridor(point.x, point.y)) {
          // 走廊内的点维持先验放行（与旧一刀切一致）：顶板/门楣投影和侧壁基
          // clutter 不能封死通道，测试 test_costmap_tunnel_roof 钉死的正是这一条。
          //
          // 走廊外的点恢复普通多帧确认 —— 这是「洞内遇敌可见化」的开口：
          // 影响区里但不在行进走廊上的真实实体，不再被无差别丢弃。
          // 判据仍是几何的、与高度无关；轴退化时保守退回整片放行。
          continue;
        }
        int map_x = 0;
        int map_y = 0;
        if (worldToMap(grid, point.x, point.y, map_x, map_y)) {
          accumulateHit(grid, map_x, map_y);
          if (collect_marked_points) {
            marked_points.push_back(point);
          }
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Failed to iterate pointcloud fields: %s", ex.what());
    }
  }

  void publishFootprint(
    const geometry_msgs::msg::TransformStamped & robot_transform, const rclcpp::Time & stamp)
  {
    if (footprint_pub_->get_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::PolygonStamped footprint;
    footprint.header.frame_id = global_frame_;
    footprint.header.stamp = stamp;
    constexpr int points_count = 32;
    footprint.polygon.points.reserve(points_count);
    for (int i = 0; i < points_count; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(points_count);
      geometry_msgs::msg::Point32 point;
      point.x = static_cast<float>(
        robot_transform.transform.translation.x + robot_radius_ * std::cos(angle));
      point.y = static_cast<float>(
        robot_transform.transform.translation.y + robot_radius_ * std::sin(angle));
      point.z = 0.0F;
      footprint.polygon.points.push_back(point);
    }
    footprint_pub_->publish(footprint);
  }

  void publishMarkedCloud(
    const std::vector<geometry_msgs::msg::Point> & points, const rclcpp::Time & stamp)
  {
    if (marked_cloud_pub_->get_subscription_count() == 0) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = global_frame_;
    cloud.header.stamp = stamp;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & point : points) {
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    marked_cloud_pub_->publish(cloud);
  }

  // 语义地图/尺寸/分辨率变化或 origin 非整数平移时整体重建。语义地图决定隧道影响区
  // （tunnel_region_），尺寸/分辨率决定格号映射，二者变化都必须重算。
  void rebuildInflationLimitCache(const nav_msgs::msg::OccupancyGrid & grid)
  {
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    const std::size_t n = grid.data.size();
    infl_limit_cache_.width = width;
    infl_limit_cache_.height = height;
    infl_limit_cache_.resolution = static_cast<double>(grid.info.resolution);
    infl_limit_cache_.origin_x = grid.info.origin.position.x;
    infl_limit_cache_.origin_y = grid.info.origin.position.y;
    infl_limit_cache_.in_tunnel.assign(n, 0);
    infl_limit_cache_.limit.assign(n, static_cast<float>(inflation_radius_));

    bool any = false;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        double world_x = 0.0;
        double world_y = 0.0;
        mapToWorld(grid, x, y, world_x, world_y);
        const TunnelSpec * spec = tunnel_region_.specNearPoint(world_x, world_y);
        if (spec == nullptr) {
          continue;
        }
        any = true;
        const std::size_t idx = gridIndex(grid, x, y);
        infl_limit_cache_.in_tunnel[idx] = 1;
        const double clearance = std::max(0.0, spec->clear_width * 0.5 - robot_radius_);
        infl_limit_cache_.limit[idx] =
          static_cast<float>(std::min(inflation_radius_, clearance));
      }
    }
    infl_limit_cache_.has_tunnel = any;
  }

  // origin 按整数格平移 (shift_x, shift_y) 后的增量更新：新格 (x,y) 复用旧格
  // (x+shift_x, y+shift_y) 的结果，只有新露出的边需要逐格算。
  void shiftInflationLimitCache(
    const nav_msgs::msg::OccupancyGrid & grid, int shift_x, int shift_y)
  {
    const int width = infl_limit_cache_.width;
    const int height = infl_limit_cache_.height;
    const std::size_t n = infl_limit_cache_.limit.size();
    std::vector<std::uint8_t> new_in_tunnel(n, 0);
    std::vector<float> new_limit(n, static_cast<float>(inflation_radius_));

    bool any = false;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int old_x = x + shift_x;
        const int old_y = y + shift_y;
        const std::size_t idx =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x);
        if (old_x >= 0 && old_x < width && old_y >= 0 && old_y < height) {
          const std::size_t old_idx =
            static_cast<std::size_t>(old_y) * static_cast<std::size_t>(width) +
            static_cast<std::size_t>(old_x);
          new_in_tunnel[idx] = infl_limit_cache_.in_tunnel[old_idx];
          new_limit[idx] = infl_limit_cache_.limit[old_idx];
        } else {
          double world_x = 0.0;
          double world_y = 0.0;
          mapToWorld(grid, x, y, world_x, world_y);
          const TunnelSpec * spec = tunnel_region_.specNearPoint(world_x, world_y);
          if (spec != nullptr) {
            new_in_tunnel[idx] = 1;
            const double clearance = std::max(0.0, spec->clear_width * 0.5 - robot_radius_);
            new_limit[idx] =
              static_cast<float>(std::min(inflation_radius_, clearance));
          }
        }
        if (new_in_tunnel[idx] != 0) {
          any = true;
        }
      }
    }

    infl_limit_cache_.in_tunnel.swap(new_in_tunnel);
    infl_limit_cache_.limit.swap(new_limit);
    infl_limit_cache_.has_tunnel = any;
    infl_limit_cache_.origin_x = grid.info.origin.position.x;
    infl_limit_cache_.origin_y = grid.info.origin.position.y;
  }

  // 本帧的逐格膨胀上限：有隧道返回缓存数组，无隧道返回空（空 = 无逐格上限）。
  const std::vector<float> & inflationLimit(const nav_msgs::msg::OccupancyGrid & grid)
  {
    static const std::vector<float> empty_limit;

    // region 为空时原实现会退回只认本体格的版本，而该版本在这些条件下同样返回空；
    // 这里直接等价为「无逐格上限」。
    if (tunnel_region_.empty() || grid.data.empty() || grid.info.resolution <= 0.0F) {
      return empty_limit;
    }

    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);
    const bool same_shape =
      infl_limit_cache_.width == width && infl_limit_cache_.height == height &&
      std::abs(infl_limit_cache_.resolution - static_cast<double>(grid.info.resolution)) <= 1e-6;

    if (!same_shape) {
      rebuildInflationLimitCache(grid);
    } else {
      const double dx =
        (grid.info.origin.position.x - infl_limit_cache_.origin_x) / infl_limit_cache_.resolution;
      const double dy =
        (grid.info.origin.position.y - infl_limit_cache_.origin_y) / infl_limit_cache_.resolution;
      const int shift_x = static_cast<int>(std::lround(dx));
      const int shift_y = static_cast<int>(std::lround(dy));
      if (std::abs(dx - static_cast<double>(shift_x)) > 1e-3 ||
        std::abs(dy - static_cast<double>(shift_y)) > 1e-3)
      {
        // 非整数格平移（如 snap_origin_to_grid 关闭）：退化为整体重建。
        rebuildInflationLimitCache(grid);
      } else if (shift_x != 0 || shift_y != 0) {
        shiftInflationLimitCache(grid, shift_x, shift_y);
      }
    }

    if (!infl_limit_cache_.has_tunnel) {
      return empty_limit;
    }
    return infl_limit_cache_.limit;
  }

  // 构建并发布进程内 Signed ESDF（有符号欧氏距离，米）：障碍外正、障碍内负、障碍
  // 表面 0。供 MINCO 障碍代价与恢复链梯度脱困查询，与 MPC 订阅的 inflated_grid
  // 同源同帧。只取 lethal(=100) 格当障碍表面：膨胀值（0-99）是衰减区，不算障碍，
  // soft 代价的 safe_dist 余量在查询方再叠加。
  //
  // 两次距离变换：第一次「障碍格为种子」得非障碍格的正距离；第二次「非障碍格为
  // 种子」得障碍格的负距离。障碍内负距离给恢复链判断「卡在多深」留了口子，即使
  // 当前只消费正距离，也把字段按 signed 语义填好，避免以后补负距离消费时再改格式。
  void publishDistanceField(
    const nav_msgs::msg::OccupancyGrid & inflated_grid, const rclcpp::Time & stamp)
  {
    if (!distance_field_enable_) {
      return;
    }
    const int w = static_cast<int>(inflated_grid.info.width);
    const int h = static_cast<int>(inflated_grid.info.height);
    if (w <= 0 || h <= 0) {
      return;
    }
    const std::size_t cell_count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    if (inflated_grid.data.size() != cell_count) {
      return;
    }
    esdf_seeds_obs_.assign(cell_count, 0);
    esdf_seeds_free_.assign(cell_count, 0);
    for (std::size_t i = 0; i < cell_count; ++i) {
      const bool obs = inflated_grid.data[i] >= 100;
      esdf_seeds_obs_[i] = obs ? 1 : 0;
      esdf_seeds_free_[i] = obs ? 0 : 1;
    }
    // 写入式 O(n) 精确 EDT；地图尺寸不变时 seeds、输出和抛物线工作区全部复用。
    exactDistanceTransform(esdf_seeds_obs_, w, h, esdf_obs_workspace_, esdf_dist_to_obs_);
    exactDistanceTransform(esdf_seeds_free_, w, h, esdf_free_workspace_, esdf_dist_to_free_);
    navigation2::DistanceFieldSnapshot snapshot;
    snapshot.resolution = static_cast<double>(inflated_grid.info.resolution);
    snapshot.origin_x = inflated_grid.info.origin.position.x;
    snapshot.origin_y = inflated_grid.info.origin.position.y;
    snapshot.width = w;
    snapshot.height = h;
    snapshot.stamp = stamp;
    snapshot.valid = true;
    snapshot.distance.resize(cell_count);
    for (std::size_t i = 0; i < cell_count; ++i) {
      const double signed_cells = esdf_seeds_obs_[i] ?
        -esdf_dist_to_free_[i] : esdf_dist_to_obs_[i];
      snapshot.distance[i] = static_cast<float>(signed_cells * snapshot.resolution);
    }
    navigation2::DistanceFieldRegistry::instance().publish(std::move(snapshot));
  }

  void updateAndPublish()
  {
    publishCurrentFootprint();

    rclcpp::Time publish_time = now();
    tf2::TimePoint reference_time = tf2::TimePointZero;
    builtin_interfaces::msg::Time observation_stamp;
    const bool have_observation_stamp = latestObservationStamp(observation_stamp);
    const bool have_new_observation = have_observation_stamp &&
      (!last_processed_observation_stamp_ ||
      !sameStamp(observation_stamp, *last_processed_observation_stamp_));
    const bool process_observations =
      !update_on_new_observation_only_ || have_new_observation;

    if (use_observation_stamp_for_origin_ && process_observations && have_observation_stamp) {
      reference_time = headerTime(observation_stamp);
      publish_time = stampToRosTime(observation_stamp);
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    if (!getRobotTransform(reference_time, robot_transform)) {
      return;
    }

    auto raw_grid = makeEmptyGrid(robot_transform, publish_time);
    seedFromPreviousGrid(raw_grid);
    std::vector<geometry_msgs::msg::Point> marked_points;
    if (process_observations && !hasFreshObservation()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No fresh observation, publishing decayed local costmap state");
    }
    const bool collect_marked_points = marked_cloud_pub_->get_subscription_count() > 0;
    if (process_observations) {
      processScan(raw_grid, marked_points, reference_time, collect_marked_points);
      // 高度判定的参考面取机器人底盘在 global_frame 下的 z，而不是 z=0 平面。
      // 定位在 z 上飘（RViz 里车沉到地面以下就是这个现象）时，整片点云在 map 系
      // 里跟着平移，拿固定的 map z 去比会让台面高度整片掉到阈值下方，高台从代价
      // 地图上静默消失。改成相对底盘的高度后，车沉、参考面跟着沉，相对高度不变。
      processPointCloud(
        raw_grid, marked_points, reference_time, collect_marked_points,
        robot_transform.transform.translation.z);
    }
    // 先验障碍合并在脚下清空之前：先验 OBSTACLE 若落在脚下，随后 markRobotFootprintFree
    // 会一并清掉，保证车体当前位置始终 free（无论障碍来自在线还是先验）。
    mergePriorObstacles(raw_grid);
    markRobotFootprintFree(raw_grid, robot_transform);

    previous_raw_grid_ = raw_grid;
    if (have_new_observation) {
      last_processed_observation_stamp_ = observation_stamp;
    }

    if (!shouldPublish(publish_time)) {
      return;
    }

    auto inflated_grid = raw_grid;
    // 逐格膨胀上限走缓存：origin 每帧整数格平移，只平移缓存 + 补边，不再逐格
    // mapToWorld + 查语义格。传影响区版本：洞口格（本体外一小圈）同样吃 clearance
    // 上限，不再被两侧墙的全量膨胀涂满。无隧道时返回空 = 无逐格上限。
    // EDT 膨胀：与卷积版逐字节等价，O(格数)，滚动窗口高频刷新的固定开销项。
    applyInflationCostGradientEDT(
      inflated_grid, inflation_radius_, 50, inflation_cost_scaling_factor_,
      inflationLimit(raw_grid));
    markRobotFootprintFree(inflated_grid, robot_transform);
    previous_inflated_grid_ = inflated_grid;

    raw_grid.header.stamp = publish_time;
    if (raw_costmap_pub_->get_subscription_count() > 0) {
      raw_costmap_pub_->publish(raw_grid);
    }
    publishMarkedCloud(marked_points, publish_time);

    inflated_grid.header.stamp = publish_time;
    costmap_pub_->publish(inflated_grid);
    publishDistanceField(inflated_grid, publish_time);
    last_publish_time_ = publish_time;
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string scan_topic_;
  std::string pointcloud_topic_;
  std::string semantic_map_topic_;
  std::string costmap_topic_;
  std::string raw_costmap_topic_;
  std::string footprint_topic_;
  std::string marked_cloud_topic_;
  bool subscribe_scan_{true};
  bool subscribe_pointcloud_{false};
  double update_frequency_{20.0};
  double publish_frequency_{10.0};
  double width_m_{5.0};
  double height_m_{5.0};
  double resolution_{0.05};
  double robot_radius_{0.25};
  double inflation_radius_{0.35};
  double inflation_cost_scaling_factor_{15.0};
  double raytrace_max_range_{6.0};
  double obstacle_min_range_{0.1};
  double obstacle_max_range_{6.0};
  double obstacle_z_min_to_robo_{0.05};
  double obstacle_z_max_to_robo_{2.0};
  double tunnel_margin_m_{0.20};
  double transform_tolerance_{0.2};
  double observation_timeout_{0.5};
  bool snap_origin_to_grid_{true};
  bool use_observation_stamp_for_origin_{false};
  bool use_latest_sensor_transform_{false};
  bool fallback_to_latest_sensor_transform_{false};
  bool update_on_new_observation_only_{false};
  bool reuse_previous_grid_{true};
  int previous_obstacle_decay_{0};
  int hit_inc_{100};
  bool distance_field_enable_{false};
  bool dynamic_decay_enable_{false};
  double dynamic_decay_keep_time_{0.8};
  double dynamic_decay_clear_time_{1.2};
  bool prior_merge_enable_{false};
  HitAgeCache hit_age_;
  // Signed ESDF 唯一生产者的复用工作区；local costmap 回调组串行，无需锁。
  std::vector<std::uint8_t> esdf_seeds_obs_;
  std::vector<std::uint8_t> esdf_seeds_free_;
  std::vector<double> esdf_dist_to_obs_;
  std::vector<double> esdf_dist_to_free_;
  DistanceTransformWorkspace esdf_obs_workspace_;
  DistanceTransformWorkspace esdf_free_workspace_;

  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_pointcloud_;
  std::optional<nav_msgs::msg::OccupancyGrid> previous_raw_grid_;
  std::optional<nav_msgs::msg::OccupancyGrid> previous_inflated_grid_;
  std::optional<builtin_interfaces::msg::Time> last_processed_observation_stamp_;
  // 未收到语义地图时 receiver_.map() 是空图，隧道查询全部退化成「没有隧道」，
  // 行为与改动前一致。
  SemanticMapReceiver receiver_;
  // 隧道影响区（本体 + tunnel_margin_m_ 边距）的查表，随语义地图重建。空表时
  // specNearPoint 恒返回 nullptr，与「没有隧道」等价。
  TunnelRegionGrid tunnel_region_;
  // 逐格膨胀上限缓存（见 InflationLimitCache 定义）。
  InflationLimitCache infl_limit_cache_;

  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marked_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmLocalCostmap)
