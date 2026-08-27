#include "grid_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "distance_transform.hpp"
#include <queue>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace navigation2
{

namespace
{

struct InflationKernelEntry
{
  int dx{0};
  int dy{0};
  int8_t cost{0};
  // 到核中心的物理距离（m）。逐格半径上限要拿它比，存下来省一次 hypot。
  float distance{0.0F};
};

const std::vector<InflationKernelEntry> & getInflationKernel(
  int radius_cells, double resolution, double cost_scaling_factor)
{
  struct KernelCache
  {
    int radius_cells{-1};
    double resolution{0.0};
    double cost_scaling_factor{0.0};
    std::vector<InflationKernelEntry> entries;
  };

  // 按参数缓存膨胀核，避免每帧重算。
  thread_local KernelCache cache;
  if (cache.radius_cells == radius_cells &&
    std::abs(cache.resolution - resolution) <= 1e-12 &&
    std::abs(cache.cost_scaling_factor - cost_scaling_factor) <= 1e-12)
  {
    return cache.entries;
  }

  cache.radius_cells = radius_cells;
  cache.resolution = resolution;
  cache.cost_scaling_factor = cost_scaling_factor;
  cache.entries.clear();
  cache.entries.reserve(
    static_cast<std::size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance =
        std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution;
      if (distance > radius_cells * resolution) {
        continue;
      }

      const double normalized = std::exp(
        -std::max(0.0, cost_scaling_factor) * std::max(0.0, distance));
      const int cost = std::clamp(
        static_cast<int>(std::round(1.0 + normalized * 98.0)), 1, 99);
      cache.entries.push_back(
        {dx, dy, static_cast<int8_t>(cost), static_cast<float>(distance)});
    }
  }

  return cache.entries;
}

}  // namespace

double normalizeAngle(double angle)
{
  // 把角度归一到 [-pi, pi]。
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

bool inBounds(const nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  return x >= 0 && y >= 0 && x < static_cast<int>(grid.info.width) &&
         y < static_cast<int>(grid.info.height);
}

std::size_t gridIndex(const nav_msgs::msg::OccupancyGrid & grid, int x, int y)
{
  return static_cast<std::size_t>(y) * grid.info.width + static_cast<std::size_t>(x);
}

bool worldToMap(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y, int & map_x,
  int & map_y)
{
  // 世界坐标转栅格坐标。
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double yaw = yawFromQuaternion(grid.info.origin.orientation);
  const double dx = world_x - origin_x;
  const double dy = world_y - origin_y;
  const double cos_yaw = std::cos(-yaw);
  const double sin_yaw = std::sin(-yaw);
  const double local_x = dx * cos_yaw - dy * sin_yaw;
  const double local_y = dx * sin_yaw + dy * cos_yaw;

  if (grid.info.resolution <= 0.0F) {
    return false;
  }
  map_x = static_cast<int>(std::floor(local_x / grid.info.resolution));
  map_y = static_cast<int>(std::floor(local_y / grid.info.resolution));
  return inBounds(grid, map_x, map_y);
}

void mapToWorld(
  const nav_msgs::msg::OccupancyGrid & grid, int map_x, int map_y, double & world_x,
  double & world_y)
{
  // 栅格中心点转回世界坐标。
  const double local_x = (static_cast<double>(map_x) + 0.5) * grid.info.resolution;
  const double local_y = (static_cast<double>(map_y) + 0.5) * grid.info.resolution;
  const double yaw = yawFromQuaternion(grid.info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  world_x = grid.info.origin.position.x + local_x * cos_yaw - local_y * sin_yaw;
  world_y = grid.info.origin.position.y + local_x * sin_yaw + local_y * cos_yaw;
}

bool isOccupied(int8_t value, int occupied_threshold, bool unknown_is_occupied)
{
  if (value < 0) {
    return unknown_is_occupied;
  }
  return value >= occupied_threshold;
}

void inflateOccupancyGrid(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold)
{
  // 直接把障碍周围一圈刷成高代价。
  if (inflation_radius <= 0.0 || grid.info.resolution <= 0.0F || grid.data.empty()) {
    return;
  }

  const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius / static_cast<double>(grid.info.resolution)));
  if (radius_cells <= 0) {
    return;
  }

  std::vector<int8_t> inflated = grid.data;
  std::vector<GridCell> occupied_cells;
  occupied_cells.reserve(grid.data.size() / 10);

  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      const auto idx = gridIndex(grid, x, y);
      if (grid.data[idx] >= occupied_threshold) {
        occupied_cells.push_back({x, y});
      }
    }
  }

  const double radius_sq = static_cast<double>(radius_cells * radius_cells);
  for (const auto & cell : occupied_cells) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (static_cast<double>(dx * dx + dy * dy) > radius_sq) {
          continue;
        }
        const int nx = cell.x + dx;
        const int ny = cell.y + dy;
        if (!inBounds(grid, nx, ny)) {
          continue;
        }
        const auto nidx = gridIndex(grid, nx, ny);
        if (inflated[nidx] >= 0 && inflated[nidx] < 100) {
          inflated[nidx] = 100;
        }
      }
    }
  }

  grid.data = std::move(inflated);
}

void applyInflationCostGradient(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold,
  double cost_scaling_factor, const std::vector<float> & radius_limit)
{
  // 用距离衰减给障碍周围铺代价梯度。
  if (inflation_radius <= 0.0 || grid.info.resolution <= 0.0F || grid.data.empty()) {
    return;
  }

  // 逐格半径上限：长度对不上就当没传，而不是索引越界。
  const bool use_limit = radius_limit.size() == grid.data.size();

  const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius / static_cast<double>(grid.info.resolution)));
  if (radius_cells <= 0) {
    return;
  }

  std::vector<int8_t> inflated = grid.data;
  std::vector<GridCell> occupied_cells;
  occupied_cells.reserve(grid.data.size() / 10);

  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      const auto idx = gridIndex(grid, x, y);
      if (grid.data[idx] >= occupied_threshold) {
        occupied_cells.push_back({x, y});
      }
    }
  }

  const auto & kernel = getInflationKernel(
    radius_cells, static_cast<double>(grid.info.resolution), cost_scaling_factor);

  for (const auto & cell : occupied_cells) {
    for (const auto & entry : kernel) {
      const int nx = cell.x + entry.dx;
      const int ny = cell.y + entry.dy;
      if (!inBounds(grid, nx, ny)) {
        continue;
      }

      const auto nidx = gridIndex(grid, nx, ny);
      if (grid.data[nidx] >= occupied_threshold || grid.data[nidx] < 0) {
        continue;
      }

      // 上限挂在「被膨胀到的格」上而不是障碍格上：隧道格只接受近处障碍的代价，
      // 洞外的墙不该把洞里涂满。壁面格自己仍然是致命的，不走这条路径。
      if (use_limit && entry.distance > radius_limit[nidx]) {
        continue;
      }

      inflated[nidx] = std::max(inflated[nidx], entry.cost);
    }
  }

  grid.data = std::move(inflated);
}

// —— EDT 膨胀：与上面两个卷积版本逐字节等价 ——
//
// 等价性论证（为什么不担心回归）：
//   卷积版的代价只依赖格间距的欧氏距离 d(n, seed)（lattice hypot × 分辨率），
//   且写的是 max 组合 ⇒ 逐格独立、与遍历顺序无关。EDT 给出的正是
//   min_seed euclid(n, seed)，于是 cost(edt(n)) 与「所有 seed 往 n 写后取
//   max」在数学上恒等；直写障碍/未知跳过、逐格半径上限这些旁路条件按原样
//   搬进来即可。等价性由 test_edt_inflation 在随机栅格上做逐字节断言兜底。
namespace
{

// 与 getInflationKernel 的公式保持同一语义（1 + 98·e^(−k·d) 夹到 [1,99]）。
inline int8_t edtInflationCost(double distance_m, double cost_scaling_factor)
{
  const double normalized = std::exp(
    -std::max(0.0, cost_scaling_factor) * std::max(0.0, distance_m));
  const int cost = std::clamp(
    static_cast<int>(std::round(1.0 + normalized * 98.0)), 1, 99);
  return static_cast<int8_t>(cost);
}

}  // namespace

void inflateOccupancyGridEDT(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold)
{
  if (inflation_radius <= 0.0 || grid.info.resolution <= 0.0F || grid.data.empty()) {
    return;
  }

  // 卷积版的有效边界是 ceil(半径/分辨率) 整格数（kernel 按格距离截断），
  // 而不是物理米数 —— 复刻同一语义才逐字节等价。
  const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius / static_cast<double>(grid.info.resolution)));
  if (radius_cells <= 0) {
    return;
  }
  const double reach_m = radius_cells * static_cast<double>(grid.info.resolution);

  // Felzenszwalb EDT 的种子是 uint8 非零：≥threshold 记种子。unknown(-1)
  // 不能当种子，先复制一份清零再喂。
  std::vector<std::uint8_t> seeds(grid.data.size());
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    seeds[i] = (grid.data[i] >= occupied_threshold && grid.data[i] <= 100) ? 1 : 0;
  }

  DistanceTransformWorkspace workspace;
  std::vector<double> distances;
  exactDistanceTransform(seeds,
    static_cast<int>(grid.info.width), static_cast<int>(grid.info.height),
    workspace, distances);

  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] >= occupied_threshold || grid.data[i] < 0) {
      continue;  // 障碍保持自身值、unknown 不动 —— 与卷积版一致。
    }
    // exactDistanceTransform 输出以「格」为单位（代价换算/上限比较时才乘分辨率，
    // 与 local_costmap 的 DistanceFieldRegistry::query 同一口径）。
    if (distances[i] <= static_cast<double>(radius_cells)) {
      grid.data[i] = 100;
    }
  }
}

void applyInflationCostGradientEDT(
  nav_msgs::msg::OccupancyGrid & grid, double inflation_radius, int occupied_threshold,
  double cost_scaling_factor, const std::vector<float> & radius_limit)
{
  if (inflation_radius <= 0.0 || grid.info.resolution <= 0.0F || grid.data.empty()) {
    return;
  }

  const bool use_limit = radius_limit.size() == grid.data.size();

  // 与 inflateOccupancyGridEDT 相同的整格化半径（卷积版语义）。
  const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius / static_cast<double>(grid.info.resolution)));
  if (radius_cells <= 0) {
    return;
  }
  const double reach_m = radius_cells * static_cast<double>(grid.info.resolution);

  std::vector<std::uint8_t> seeds(grid.data.size());
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    seeds[i] = (grid.data[i] >= occupied_threshold && grid.data[i] <= 100) ? 1 : 0;
  }

  DistanceTransformWorkspace workspace;
  std::vector<double> distances;
  exactDistanceTransform(seeds,
    static_cast<int>(grid.info.width), static_cast<int>(grid.info.height),
    workspace, distances);

  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] >= occupied_threshold || grid.data[i] < 0) {
      continue;  // 壁面/未知不覆盖。
    }
    const double dist_m = distances[i] * grid.info.resolution;
    if (dist_m > reach_m) {
      continue;
    }
    // 上限挂在「被膨胀到的格」上而不是障碍格上（隧道格只接受近处障碍的代价，
    // 洞外的墙不该把洞里涂满）——条件与卷积版逐字一致。
    if (use_limit && static_cast<float>(dist_m) > radius_limit[i]) {
      continue;
    }
    grid.data[i] = std::max(grid.data[i], edtInflationCost(dist_m, cost_scaling_factor));
  }
}

std::vector<GridCell> raytraceLine(int x0, int y0, int x1, int y1)
{
  std::vector<GridCell> cells;
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;

  while (true) {
    cells.push_back({x, y});
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
  return cells;
}

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z)
{
  tf2::Transform tf_transform;
  tf2::fromMsg(transform.transform, tf_transform);
  const tf2::Vector3 point = tf_transform * tf2::Vector3(x, y, z);

  geometry_msgs::msg::Point out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z();
  return out;
}

PlanarFrame makePlanarFrame(const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform tf_transform;
  tf2::fromMsg(transform.transform, tf_transform);
  const tf2::Matrix3x3 & R = tf_transform.getBasis();
  PlanarFrame f;
  f.tx = transform.transform.translation.x;
  f.ty = transform.transform.translation.y;
  f.tz = transform.transform.translation.z;
  f.m20 = R[2][0]; f.m21 = R[2][1]; f.m22 = R[2][2];
  // 平面投影只需要 yaw 行：R[0]/R[1] 的前两列合成 cos/sin。
  f.cos_yaw = R[0][0];
  f.sin_yaw = R[1][0];
  return f;
}

geometry_msgs::msg::Point applyPlanarFrame(const PlanarFrame & f, double x, double y, double z)
{
  geometry_msgs::msg::Point out;
  out.x = f.cos_yaw * x - f.sin_yaw * y + f.tx;
  out.y = f.sin_yaw * x + f.cos_yaw * y + f.ty;
  out.z = f.m20 * x + f.m21 * y + f.m22 * z + f.tz;
  return out;
}

geometry_msgs::msg::Point transformPointPlanar(
  const geometry_msgs::msg::TransformStamped & transform, double x, double y, double z)
{
  const double yaw = yawFromQuaternion(transform.transform.rotation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  geometry_msgs::msg::Point out;
  out.x = transform.transform.translation.x + x * cos_yaw - y * sin_yaw;
  out.y = transform.transform.translation.y + x * sin_yaw + y * cos_yaw;
  out.z = transform.transform.translation.z + z;
  return out;
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame_id, const rclcpp::Time & stamp, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = stamp;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = quaternionFromYaw(yaw);
  return pose;
}

}  // namespace navigation2
