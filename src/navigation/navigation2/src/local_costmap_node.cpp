#include "grid_utils.hpp"
#include "semantic_map_consumer.hpp"

#include <algorithm>
#include <cmath>
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

class RmLocalCostmap : public rclcpp::Node
{
public:
  explicit RmLocalCostmap(const rclcpp::NodeOptions & options)
  : Node("rm_local_costmap", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    // 按开关订阅不同传感器输入。
    if (subscribe_scan_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          latest_scan_ = std::move(msg);
        });
    }
    if (subscribe_pointcloud_) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          latest_pointcloud_ = std::move(msg);
        });
    }

    // 语义地图：局部代价地图自己不加载 /map，但隧道顶板必须在这里也被滤掉 ——
    // 局部图才是 MPC 的碰撞依据，只在全局图上放行等于让车看见洞口却撞在顶板上。
    semantic_map_sub_ = create_subscription<decision_interfaces::msg::SemanticMap>(
      semantic_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](decision_interfaces::msg::SemanticMap::SharedPtr msg) {
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
        RCLCPP_INFO(
          get_logger(), "Local costmap got semantic map: %zu tunnels",
          receiver_.map().tunnels().size());
      });

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
      });

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
    // 隧道本体内的高度带上限，同样相对底盘。语义与 global_costmap_node.cpp 的同名
    // 参数一致，两边必须一起改。
    tunnel_obstacle_z_max_to_robo_ =
      declare_parameter<double>("tunnel_obstacle_z_max_to_robo", 0.20);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);
    observation_timeout_ = declare_parameter<double>("observation_timeout", 0.5);
    snap_origin_to_grid_ = declare_parameter<bool>("snap_origin_to_grid", true);
    use_observation_stamp_for_origin_ =
      declare_parameter<bool>("use_observation_stamp_for_origin", false);
    publish_on_observation_update_ =
      declare_parameter<bool>("publish_on_observation_update", false);
    use_latest_sensor_transform_ =
      declare_parameter<bool>("use_latest_sensor_transform", false);
    fallback_to_latest_sensor_transform_ =
      declare_parameter<bool>("fallback_to_latest_sensor_transform", false);
    update_on_new_observation_only_ =
      declare_parameter<bool>("update_on_new_observation_only", false);
    reuse_previous_grid_ = declare_parameter<bool>("reuse_previous_grid", true);
    previous_obstacle_decay_ = declare_parameter<int>("previous_obstacle_decay", 0);
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
    return age <= observation_timeout_;
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

  void publishCachedCostmap(const rclcpp::Time & stamp)
  {
    if (!previous_raw_grid_ || !previous_inflated_grid_ || !shouldPublish(stamp)) {
      return;
    }

    auto raw_grid = *previous_raw_grid_;
    raw_grid.header.stamp = stamp;
    if (raw_costmap_pub_->get_subscription_count() > 0) {
      raw_costmap_pub_->publish(raw_grid);
    }

    auto inflated_grid = *previous_inflated_grid_;
    inflated_grid.header.stamp = stamp;
    costmap_pub_->publish(inflated_grid);
    last_publish_time_ = stamp;
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
      return;
    }

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
        grid.data[gridIndex(grid, endpoint.x, endpoint.y)] = 100;
      }
    }
  }

  // 该点所在位置允许的最大障碍高度（相对底盘）。隧道本体内压到
  // tunnel_obstacle_z_max_to_robo_，把顶板滤掉；该阈值以下的真障碍照常标记。
  //
  // 阈值不取 TunnelSpec::clear_height：净高从地面量，这里的高度相对 base_link，
  // 差一个未知的底盘离地偏置。语义与 global_costmap_node.cpp 的同名函数一致，两边
  // 必须一起改 —— 只在一边放行会让全局规划出的路在局部层被判为撞墙。
  double tunnelHeightLimit(double world_x, double world_y) const
  {
    if (tunnelSpecAtPoint(receiver_.map(), world_x, world_y) == nullptr) {
      return std::numeric_limits<double>::infinity();
    }
    return tunnel_obstacle_z_max_to_robo_;
  }

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

        const auto point = transformPoint(cloud_to_global, *iter_x, *iter_y, *iter_z);
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
        if (height_to_robo > tunnelHeightLimit(point.x, point.y)) {
          continue;
        }
        int map_x = 0;
        int map_y = 0;
        if (worldToMap(grid, point.x, point.y, map_x, map_y)) {
          grid.data[gridIndex(grid, map_x, map_y)] = 100;
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
    markRobotFootprintFree(raw_grid, robot_transform);

    previous_raw_grid_ = raw_grid;
    if (have_new_observation) {
      last_processed_observation_stamp_ = observation_stamp;
    }

    if (!shouldPublish(publish_time)) {
      return;
    }

    auto inflated_grid = raw_grid;
    // 局部栅格跟车滚动，origin 每帧都变，逐格上限不能跨帧缓存。地图里没有隧道时
    // makeInflationRadiusLimit 直接返回空，不用逐格扫。
    applyInflationCostGradient(
      inflated_grid, inflation_radius_, 50, inflation_cost_scaling_factor_,
      makeInflationRadiusLimit(raw_grid, receiver_.map(), inflation_radius_, robot_radius_));
    markRobotFootprintFree(inflated_grid, robot_transform);
    previous_inflated_grid_ = inflated_grid;

    raw_grid.header.stamp = publish_time;
    if (raw_costmap_pub_->get_subscription_count() > 0) {
      raw_costmap_pub_->publish(raw_grid);
    }
    publishMarkedCloud(marked_points, publish_time);

    inflated_grid.header.stamp = publish_time;
    costmap_pub_->publish(inflated_grid);
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
  double resolution_{0.02};
  double robot_radius_{0.25};
  double inflation_radius_{0.6};
  double inflation_cost_scaling_factor_{5.0};
  double raytrace_max_range_{6.0};
  double obstacle_min_range_{0.1};
  double obstacle_max_range_{6.0};
  double obstacle_z_min_to_robo_{0.05};
  double obstacle_z_max_to_robo_{2.0};
  double tunnel_obstacle_z_max_to_robo_{0.20};
  double transform_tolerance_{0.2};
  double observation_timeout_{0.5};
  bool snap_origin_to_grid_{true};
  bool use_observation_stamp_for_origin_{false};
  bool publish_on_observation_update_{false};
  bool use_latest_sensor_transform_{false};
  bool fallback_to_latest_sensor_transform_{false};
  bool update_on_new_observation_only_{false};
  bool reuse_previous_grid_{true};
  int previous_obstacle_decay_{0};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  std::optional<nav_msgs::msg::OccupancyGrid> previous_raw_grid_;
  std::optional<nav_msgs::msg::OccupancyGrid> previous_inflated_grid_;
  std::optional<builtin_interfaces::msg::Time> last_processed_observation_stamp_;
  // 未收到语义地图时 receiver_.map() 是空图，隧道查询全部退化成「没有隧道」，
  // 行为与改动前一致。
  SemanticMapReceiver receiver_;

  rclcpp::Subscription<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_sub_;
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
