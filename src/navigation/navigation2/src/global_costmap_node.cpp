#include "grid_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

class RmGlobalCostmap : public rclcpp::Node
{
public:
  explicit RmGlobalCostmap(const rclcpp::NodeOptions & options)
  : Node("rm_global_costmap", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    loadParameters();

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = std::move(msg);
        publishCostmap();
      });

    // 点云和激光都可作为全局障碍输入。
    if (subscribe_pointcloud_) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          latest_pointcloud_ = std::move(msg);
        });
    }
    if (subscribe_scan_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          latest_scan_ = std::move(msg);
        });
    }

    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    raw_costmap_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(raw_costmap_topic_, costmap_qos);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, costmap_qos);
    footprint_pub_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(footprint_topic_, rclcpp::QoS(1));
    marked_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(marked_cloud_topic_, rclcpp::QoS(1));
    voxel_grid_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(voxel_grid_topic_, rclcpp::QoS(1));

    // 周期发布给下游规划器用。
    if (publish_frequency_ > 0.0) {
      const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_frequency_));
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          publishCostmap();
        });
    }

    RCLCPP_INFO(
      get_logger(), "rm_global_costmap ready: map=%s pointcloud=%s output=%s",
      map_topic_.c_str(), subscribe_pointcloud_ ? pointcloud_topic_.c_str() : "disabled",
      costmap_topic_.c_str());
  }

private:
  void loadParameters()
  {
    // 基本坐标系和话题。
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link_fake");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    pointcloud_topic_ =
      declare_parameter<std::string>("pointcloud_topic", "/segmentation/obstacle");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    raw_costmap_topic_ =
      declare_parameter<std::string>("raw_costmap_topic", "/global_costmap/costmap_raw");
    footprint_topic_ =
      declare_parameter<std::string>("footprint_topic", "/global_costmap/published_footprint");
    marked_cloud_topic_ =
      declare_parameter<std::string>("marked_cloud_topic", "/global_costmap/voxel_marked_cloud");
    voxel_grid_topic_ =
      declare_parameter<std::string>("voxel_grid_topic", "/global_costmap/voxel_grid");
    subscribe_pointcloud_ = declare_parameter<bool>("subscribe_pointcloud", true);
    subscribe_scan_ = declare_parameter<bool>("subscribe_scan", false);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    publish_frequency_ = declare_parameter<double>("publish_frequency", 2.0);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.7);
    inflation_cost_scaling_factor_ =
      declare_parameter<double>("inflation_cost_scaling_factor", 8.0);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    obstacle_min_range_ = declare_parameter<double>("obstacle_min_range", 0.1);
    obstacle_max_range_ = declare_parameter<double>("obstacle_max_range", 3.0);
    // 障碍高度带，相对机器人底盘（base_link）而非固定的 map z=0 平面。
    // map 系的 z=0 是雷达平面不是地面，详见 local_costmap_node.cpp 里的说明。
    obstacle_z_min_to_robo_ = declare_parameter<double>("obstacle_z_min_to_robo", 0.1);
    obstacle_z_max_to_robo_ = declare_parameter<double>("obstacle_z_max_to_robo", 2.0);
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.2);
    observation_timeout_ = declare_parameter<double>("observation_timeout", 1.0);
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

  bool observationIsFresh(const builtin_interfaces::msg::Time & stamp) const
  {
    if (observation_timeout_ <= 0.0 || stampIsZero(stamp)) {
      return true;
    }

    const auto age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age <= observation_timeout_;
  }

  bool getRobotTransform(geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot get robot transform %s -> %s: %s",
        global_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return false;
    }
  }

  bool gridCompatible(
    const nav_msgs::msg::OccupancyGrid & lhs,
    const nav_msgs::msg::OccupancyGrid & rhs) const
  {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width &&
           lhs.info.height == rhs.info.height &&
           std::abs(lhs.info.resolution - rhs.info.resolution) <= 1e-6 &&
           std::abs(lhs.info.origin.position.x - rhs.info.origin.position.x) <= 1e-6 &&
           std::abs(lhs.info.origin.position.y - rhs.info.origin.position.y) <= 1e-6 &&
           std::abs(yawFromQuaternion(lhs.info.origin.orientation) -
             yawFromQuaternion(rhs.info.origin.orientation)) <= 1e-6;
  }

  void seedFromPreviousGrid(nav_msgs::msg::OccupancyGrid & grid)
  {
    // 旧障碍可以按需延续一帧。
    if (!reuse_previous_grid_ || !previous_raw_grid_ ||
      !gridCompatible(*previous_raw_grid_, grid))
    {
      return;
    }

    const auto & previous = *previous_raw_grid_;
    for (unsigned int y = 0; y < grid.info.height; ++y) {
      for (unsigned int x = 0; x < grid.info.width; ++x) {
        const auto index = gridIndex(grid, static_cast<int>(x), static_cast<int>(y));
        int persisted_cost = static_cast<int>(previous.data[index]);
        if (persisted_cost <= 0) {
          continue;
        }

        persisted_cost = std::max(0, persisted_cost - previous_obstacle_decay_);
        if (persisted_cost <= 0) {
          continue;
        }

        grid.data[index] = std::max(grid.data[index], static_cast<int8_t>(persisted_cost));
      }
    }
  }

  // 把激光点云/扫描的命中点标成全局代价地图里的障碍。
  // 比局部代价地图更简单：不做射线清除，只标记命中格子。
  void processScan(
    nav_msgs::msg::OccupancyGrid & grid,
    std::vector<geometry_msgs::msg::Point> & marked_points,
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
    try {
      scan_to_global = tf_buffer_->lookupTransform(
        global_frame_, scan_frame, headerTime(scan.header.stamp),
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform scan frame %s to %s: %s",
        scan_frame.c_str(), global_frame_.c_str(), ex.what());
      return;
    }

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r < obstacle_min_range_ || r > obstacle_max_range_) {
        continue;
      }
      const double angle = scan.angle_min +
        static_cast<double>(i) * scan.angle_increment;
      const auto endpoint =
        transformPoint(scan_to_global, r * std::cos(angle), r * std::sin(angle), 0.0);

      int map_x = 0;
      int map_y = 0;
      if (worldToMap(grid, endpoint.x, endpoint.y, map_x, map_y)) {
        grid.data[gridIndex(grid, map_x, map_y)] = 100;
        if (collect_marked_points) {
          marked_points.push_back(endpoint);
        }
      }
    }
  }

  void processPointCloud(
    nav_msgs::msg::OccupancyGrid & grid,
    std::vector<geometry_msgs::msg::Point> & marked_points,
    const bool collect_marked_points,
    const double robo_z)
  {
    if (!latest_pointcloud_) {
      return;
    }

    const auto & cloud = *latest_pointcloud_;
    if (!observationIsFresh(cloud.header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Latest global costmap pointcloud is stale, ignore it");
      return;
    }

    const std::string cloud_frame =
      cloud.header.frame_id.empty() ? robot_base_frame_ : cloud.header.frame_id;

    geometry_msgs::msg::TransformStamped cloud_to_global;
    try {
      cloud_to_global = tf_buffer_->lookupTransform(
        global_frame_, cloud_frame, headerTime(cloud.header.stamp),
        tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform pointcloud frame %s to %s at message stamp: %s",
        cloud_frame.c_str(), global_frame_.c_str(), ex.what());
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
        // 高度判定取「相对机器人底盘」的差值。map 系的 z=0 是雷达平面不是地面
        // （Super-LIO 的 odom 原点锚在开机瞬间的雷达位姿上），拿绝对 z 比常数会让
        // 20cm 高台静默消失。做差之后雷达平面偏移和定位 z 漂移都被抵消。
        // 详细推导见 local_costmap_node.cpp，两边必须保持一致。
        const double height_to_robo = point.z - robo_z;
        if (height_to_robo < obstacle_z_min_to_robo_ ||
          height_to_robo > obstacle_z_max_to_robo_)
        {
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

  void publishFootprint(const rclcpp::Time & stamp)
  {
    if (footprint_pub_->get_subscription_count() == 0) {
      return;
    }

    geometry_msgs::msg::TransformStamped robot_transform;
    if (!getRobotTransform(robot_transform)) {
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

  sensor_msgs::msg::PointCloud2 makeMarkedCloud(
    const std::vector<geometry_msgs::msg::Point> & points, const rclcpp::Time & stamp) const
  {
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

    return cloud;
  }

  void publishCostmap()
  {
    if (!latest_map_) {
      return;
    }

    const auto now_time = now();
    nav_msgs::msg::OccupancyGrid raw_grid = *latest_map_;
    if (raw_grid.header.frame_id.empty()) {
      raw_grid.header.frame_id = global_frame_;
    }
    raw_grid.header.stamp = now_time;

    seedFromPreviousGrid(raw_grid);
    std::vector<geometry_msgs::msg::Point> marked_points;
    const bool collect_marked_points =
      marked_cloud_pub_->get_subscription_count() > 0 ||
      voxel_grid_pub_->get_subscription_count() > 0;
    processScan(raw_grid, marked_points, collect_marked_points);
    // 高度判定的参考面：机器人底盘在 global_frame 下的 z（rose 的 robo_z）。
    // 拿不到 TF 时退回 0.0。
    double robo_z = 0.0;
    geometry_msgs::msg::TransformStamped robo_transform;
    if (getRobotTransform(robo_transform)) {
      robo_z = robo_transform.transform.translation.z;
    }
    processPointCloud(raw_grid, marked_points, collect_marked_points, robo_z);
    previous_raw_grid_ = raw_grid;

    if (raw_costmap_pub_->get_subscription_count() > 0) {
      raw_costmap_pub_->publish(raw_grid);
    }
    publishFootprint(now_time);

    if (marked_cloud_pub_->get_subscription_count() > 0 ||
      voxel_grid_pub_->get_subscription_count() > 0)
    {
      const auto marked_cloud = makeMarkedCloud(marked_points, now_time);
      if (marked_cloud_pub_->get_subscription_count() > 0) {
        marked_cloud_pub_->publish(marked_cloud);
      }
      if (voxel_grid_pub_->get_subscription_count() > 0) {
        voxel_grid_pub_->publish(marked_cloud);
      }
    }

    auto inflated_grid = raw_grid;
    applyInflationCostGradient(
      inflated_grid, inflation_radius_, occupied_threshold_, inflation_cost_scaling_factor_);
    inflated_grid.header.stamp = now_time;
    costmap_pub_->publish(inflated_grid);
  }

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string map_topic_;
  std::string pointcloud_topic_;
  std::string costmap_topic_;
  std::string raw_costmap_topic_;
  std::string footprint_topic_;
  std::string marked_cloud_topic_;
  std::string voxel_grid_topic_;
  bool subscribe_pointcloud_{true};
  bool subscribe_scan_{false};
  std::string scan_topic_;
  double publish_frequency_{2.0};
  double robot_radius_{0.25};
  double inflation_radius_{0.7};
  double inflation_cost_scaling_factor_{8.0};
  int occupied_threshold_{50};
  double obstacle_min_range_{0.1};
  double obstacle_max_range_{3.0};
  double obstacle_z_min_to_robo_{0.1};
  double obstacle_z_max_to_robo_{2.0};
  double transform_tolerance_{0.2};
  double observation_timeout_{1.0};
  bool reuse_previous_grid_{true};
  int previous_obstacle_decay_{0};

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  std::optional<nav_msgs::msg::OccupancyGrid> previous_raw_grid_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marked_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmGlobalCostmap)
