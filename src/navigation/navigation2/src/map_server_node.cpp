#include "grid_utils.hpp"
#include "semantic_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <decision_interfaces/msg/semantic_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace navigation2
{

// 语义地图服务器。
//
// 地图文件是 msgpack，它是唯一真源：/map 的占据栅格由 terrain 通道里的 OBSTACLE
// 推导出来，所以语义层和占据层不可能不同步 —— 这是相对「pgm 管占据、另一个文件
// 管语义」的关键改进，那种做法下两份文件各自被编辑过就会静默错位。
//
// 膨胀在这里离线做完再发。这不是省 CPU：MINCO 是 L-BFGS、MPC 是 OSQP，都靠梯度
// 工作。代价在障碍格内跳到 255、格外是 0，梯度既不连续（线搜索失败）又在一格之
// 外恒为零（优化器根本看不见远处的约束）。摊成连续可导的场，约束才能以梯度的
// 形式传播出去。
class RmMapServer : public rclcpp::Node
{
public:
  explicit RmMapServer(const rclcpp::NodeOptions & options)
  : Node("rm_map_server", options)
  {
    map_filename_ = declare_parameter<std::string>("map_filename", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    semantic_map_topic_ =
      declare_parameter<std::string>("semantic_map_topic", "/map_server/semantic_map");
    publish_frequency_ = declare_parameter<double>("publish_frequency", 1.0);

    // 膨胀参数用物理距离而不是格数：换分辨率时不用重新调参。
    inflation_.full_cost_radius_m =
      declare_parameter<double>("inflation.full_cost_radius_m", 0.10);
    inflation_.cutoff_radius_m = declare_parameter<double>("inflation.cutoff_radius_m", 0.30);
    inflation_.decay_rate_per_m = declare_parameter<double>("inflation.decay_rate_per_m", 24.0);
    inflation_.non_body_magnitude_cap = declare_parameter<double>(
      "inflation.direction_non_body_magnitude_cap", kMaxInflatedMagnitude);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);
    semantic_map_pub_ =
      create_publisher<decision_interfaces::msg::SemanticMap>(semantic_map_topic_, qos);
    reload_srv_ = create_service<std_srvs::srv::Empty>(
      "~/reload_map",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        loadAndPublish();
      });

    // 按需定时重发地图，方便晚启动的订阅者接上。
    if (publish_frequency_ > 0.0) {
      const auto period =
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_frequency_));
      publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          if (has_map_) {
            map_pub_->publish(map_);
            semantic_map_pub_->publish(semantic_msg_);
          }
        });
    }

    loadAndPublish();
  }

private:
  void loadAndPublish()
  {
    get_parameter("map_filename", map_filename_);
    if (map_filename_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No map_filename was provided. rm_map_server will wait until the parameter is set.");
      has_map_ = false;
      return;
    }

    try {
      const SemanticMapData data = loadSemanticMap(map_filename_);
      inflation_.resolution = data.geometry.resolution;
      const SemanticMap map = SemanticMap::inflate(data, inflation_);

      map_ = makeOccupancyGrid(map);
      semantic_msg_ = makeSemanticMsg(map);
      has_map_ = true;
      map_pub_->publish(map_);
      semantic_map_pub_->publish(semantic_msg_);

      std::size_t tunnel_cells = 0;
      std::size_t unknown_cells = 0;
      for (const std::uint8_t label : map.terrain()) {
        if (label == static_cast<std::uint8_t>(TerrainType::TUNNEL)) {
          ++tunnel_cells;
        } else if (label == static_cast<std::uint8_t>(TerrainType::UNKNOWN)) {
          ++unknown_cells;
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "Loaded semantic map %s (%dx%d, %.3f m/cell, origin [%.2f, %.2f]): "
        "%zu tunnel cells in %zu tunnels, %zu unknown cells",
        map_filename_.c_str(), map.geometry().width, map.geometry().height,
        map.geometry().resolution, map.geometry().origin.x(), map.geometry().origin.y(),
        tunnel_cells, map.tunnels().size(), unknown_cells);
      for (std::size_t i = 0; i < map.tunnels().size(); ++i) {
        const TunnelSpec & spec = map.tunnels()[i];
        RCLCPP_INFO(
          get_logger(),
          "  tunnel %zu: clear %.2f m high x %.2f m wide, run-up %.2f m, speed %.2f~%.2f m/s",
          i + 1, spec.clear_height, spec.clear_width, spec.run_up, spec.velocity_min,
          spec.velocity_max);
      }
    } catch (const std::exception & ex) {
      has_map_ = false;
      RCLCPP_ERROR(get_logger(), "Failed to load map: %s", ex.what());
    }
  }

  nav_msgs::msg::OccupancyGrid makeOccupancyGrid(const SemanticMap & map) const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = now();
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(map.geometry().resolution);
    grid.info.width = static_cast<unsigned int>(map.geometry().width);
    grid.info.height = static_cast<unsigned int>(map.geometry().height);
    grid.info.origin.position.x = map.geometry().origin.x();
    grid.info.origin.position.y = map.geometry().origin.y();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation = quaternionFromYaw(0.0);

    grid.data.resize(map.terrain().size());
    for (std::size_t i = 0; i < map.terrain().size(); ++i) {
      switch (static_cast<TerrainType>(map.terrain()[i])) {
        case TerrainType::OBSTACLE:
          grid.data[i] = 100;
          break;
        case TerrainType::UNKNOWN:
          // 未知必须回到 -1。它不是空闲：这些格子从未扫到，A* 要对它们收
          // unknown_cost，当成空闲会让规划器以正常代价穿越从未观测的区域。
          grid.data[i] = -1;
          break;
        case TerrainType::TUNNEL:
          // 隧道在占据层里是空闲的 —— 它能过。「必须摆对姿态才能过」这件事由
          // 语义层表达，占据栅格没有位置放它。
          grid.data[i] = 0;
          break;
        default:
          grid.data[i] = 0;
          break;
      }
    }
    return grid;
  }

  decision_interfaces::msg::SemanticMap makeSemanticMsg(const SemanticMap & map) const
  {
    decision_interfaces::msg::SemanticMap msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.width = static_cast<std::uint32_t>(map.geometry().width);
    msg.height = static_cast<std::uint32_t>(map.geometry().height);
    msg.resolution = map.geometry().resolution;
    msg.origin_x = map.geometry().origin.x();
    msg.origin_y = map.geometry().origin.y();
    msg.terrain = map.terrain();
    msg.cost = map.cost();

    const std::size_t cells = map.terrain().size();
    msg.direction_angle.resize(cells);
    msg.direction_magnitude.resize(cells);
    for (int y = 0; y < map.geometry().height; ++y) {
      for (int x = 0; x < map.geometry().width; ++x) {
        const std::size_t index = map.geometry().index(x, y);
        const Eigen::Vector2d direction = map.directionAtCell(x, y);
        const double magnitude = direction.norm();
        if (magnitude < 1e-12) {
          msg.direction_angle[index] = 0;
          msg.direction_magnitude[index] = 0;
          continue;
        }
        double angle = std::atan2(direction.y(), direction.x());
        if (angle < 0.0) {
          angle += 2.0 * M_PI;
        }
        msg.direction_angle[index] = static_cast<std::uint8_t>(
          std::clamp(std::lround(angle / (2.0 * M_PI) * 255.0), 0L, 255L));
        msg.direction_magnitude[index] = static_cast<std::uint8_t>(
          std::clamp(std::lround(std::min(magnitude, 1.0) * 255.0), 0L, 255L));
      }
    }

    msg.tunnels.reserve(map.tunnels().size());
    for (const TunnelSpec & spec : map.tunnels()) {
      decision_interfaces::msg::TunnelSpec entry;
      entry.clear_height = spec.clear_height;
      entry.clear_width = spec.clear_width;
      entry.run_up = spec.run_up;
      entry.velocity_min = spec.velocity_min;
      entry.velocity_max = spec.velocity_max;
      msg.tunnels.push_back(entry);
    }
    msg.tunnel_ids = map.tunnelIds();
    return msg;
  }

  std::string map_filename_;
  std::string frame_id_;
  std::string map_topic_;
  std::string semantic_map_topic_;
  double publish_frequency_{1.0};
  InflationParams inflation_;
  bool has_map_{false};
  nav_msgs::msg::OccupancyGrid map_;
  decision_interfaces::msg::SemanticMap semantic_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<decision_interfaces::msg::SemanticMap>::SharedPtr semantic_map_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reload_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace navigation2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation2::RmMapServer)
