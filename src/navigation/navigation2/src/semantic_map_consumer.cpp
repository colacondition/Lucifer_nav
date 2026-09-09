#include "semantic_map_consumer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "grid_utils.hpp"

namespace navigation2
{

SemanticMap semanticMapFromMsg(const decision_interfaces::msg::SemanticMap & msg)
{
  GridGeometry geometry;
  geometry.width = static_cast<int>(msg.width);
  geometry.height = static_cast<int>(msg.height);
  geometry.resolution = msg.resolution;
  geometry.origin = Eigen::Vector2d(msg.origin_x, msg.origin_y);
  if (!geometry.valid()) {
    throw std::runtime_error(
            "semantic map message has invalid geometry: " + std::to_string(msg.width) + "x" +
            std::to_string(msg.height) + " @ " + std::to_string(msg.resolution) + " m/cell");
  }

  std::vector<TunnelSpec> tunnels;
  tunnels.reserve(msg.tunnels.size());
  for (const auto & entry : msg.tunnels) {
    TunnelSpec spec;
    spec.clear_height = entry.clear_height;
    spec.clear_width = entry.clear_width;
    spec.run_up = entry.run_up;
    spec.velocity_min = entry.velocity_min;
    spec.velocity_max = entry.velocity_max;
    tunnels.push_back(spec);
  }

  // fromChannels 会校验各通道长度是否与几何一致，不一致时抛。
  return SemanticMap::fromChannels(
    geometry, msg.terrain, msg.direction_angle, msg.direction_magnitude, msg.cost,
    std::move(tunnels), msg.tunnel_ids);
}

namespace
{

// 只比会影响语义的字段。header 每次重发都不同，比上它等于禁用缓存。
bool sameContent(
  const decision_interfaces::msg::SemanticMap & a,
  const decision_interfaces::msg::SemanticMap & b)
{
  return a.width == b.width && a.height == b.height && a.resolution == b.resolution &&
         a.origin_x == b.origin_x && a.origin_y == b.origin_y && a.terrain == b.terrain &&
         a.direction_angle == b.direction_angle &&
         a.direction_magnitude == b.direction_magnitude && a.cost == b.cost &&
         a.tunnel_ids == b.tunnel_ids && a.tunnels == b.tunnels;
}

}  // namespace

bool SemanticMapReceiver::update(const decision_interfaces::msg::SemanticMap & msg)
{
  if (has_map_ && sameContent(last_msg_, msg)) {
    return false;
  }
  // 先解析再落地：解析抛异常时 map_ 保持上一张好图不变。
  SemanticMap parsed = semanticMapFromMsg(msg);
  map_ = std::move(parsed);
  last_msg_ = msg;
  has_map_ = true;
  return true;
}

const TunnelSpec * tunnelSpecAtPoint(const SemanticMap & map, double world_x, double world_y)
{
  if (!map.valid()) {
    return nullptr;
  }
  const auto cell = map.geometry().containingCell(Eigen::Vector2d(world_x, world_y));
  if (!cell) {
    return nullptr;
  }
  return map.tunnelSpecAtCell(cell->x(), cell->y());
}

bool tunnelAxisAtPoint(const SemanticMap & map, const Eigen::Vector2d & point, Eigen::Vector2d & axis)
{
  if (!map.valid()) {
    return false;
  }
  const auto cell = map.geometry().containingCell(point);
  if (!cell) {
    return false;
  }
  // 只认本体格：膨胀圈的方向是弱信息，拿它去拧平滑后的走向会在洞口外就开始加代价。
  if (!map.isTunnelBodyCell(cell->x(), cell->y())) {
    return false;
  }
  const Eigen::Vector2d raw = map.directionAtCell(cell->x(), cell->y());
  const double norm = raw.norm();
  if (norm < 1e-9) {
    return false;
  }
  axis = raw / norm;
  return true;
}

std::vector<float> makeInflationRadiusLimit(
  const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map, double default_radius,
  double robot_radius)
{
  // 没有隧道就没有逐格上限可言。提前退出让「地图里没隧道」的常见情形不用逐格扫，
  // 局部代价地图每帧都会调这个函数。
  if (!map.valid() || map.tunnels().empty() || grid.data.empty() || grid.info.resolution <= 0.0F) {
    return {};
  }

  std::vector<float> limits(grid.data.size(), static_cast<float>(default_radius));
  bool any_tunnel = false;

  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      double world_x = 0.0;
      double world_y = 0.0;
      mapToWorld(grid, x, y, world_x, world_y);
      const TunnelSpec * spec = tunnelSpecAtPoint(map, world_x, world_y);
      if (spec == nullptr) {
        continue;
      }
      any_tunnel = true;
      // 洞内可用的横向余量：净宽的一半减去车宽的一半。膨胀不能超过它，否则通道
      // 中线也会被涂上代价。取 0 下限是因为「洞比车还窄」时该由变形后的车宽去
      // 保证能过，膨胀层不该在这里替规划器做否决 —— 真过不去会在壁面的致命格上
      // 挡住，而不是靠膨胀。
      const double clearance = std::max(0.0, spec->clear_width * 0.5 - robot_radius);
      limits[gridIndex(grid, x, y)] =
        static_cast<float>(std::min(default_radius, clearance));
    }
  }

  if (!any_tunnel) {
    return {};
  }
  return limits;
}

TunnelRegionGrid TunnelRegionGrid::build(const SemanticMap & map, double margin_m)
{
  TunnelRegionGrid result;
  if (!map.valid() || map.tunnels().empty()) {
    return result;
  }

  const auto & geometry = map.geometry();
  const int width = geometry.width;
  const int height = geometry.height;
  const std::size_t cells = geometry.cellCount();

  // 边距按物理距离换成格数，向上取整（截断会让恰好在边距上的格掉出影响区，
  // 与 nearestTunnelBody 的 span 取整理由相同）。margin_m <= 0 时退化成只有本体格。
  const int span = std::max(
    0, static_cast<int>(std::ceil(std::max(0.0, margin_m) / geometry.resolution)));
  const double margin_sq_m =
    std::max(0.0, margin_m) * std::max(0.0, margin_m);

  std::vector<std::uint8_t> spec_index(cells, 0);
  // 多条隧道的边距可能重叠，逐格记录到最近本体格的距离，近者胜。
  std::vector<float> best_sq(cells, std::numeric_limits<float>::infinity());
  bool any = false;

  // 走廊轴的原料：每条隧道的本体格世界坐标集合。
  std::vector<std::vector<Eigen::Vector2d>> body_points(map.tunnels().size());

  for (int sy = 0; sy < height; ++sy) {
    for (int sx = 0; sx < width; ++sx) {
      if (!map.isTunnelBodyCell(sx, sy)) {
        continue;
      }
      const TunnelSpec * spec = map.tunnelSpecAtCell(sx, sy);
      if (spec == nullptr) {
        continue;
      }
      any = true;
      // tunnelSpecAtCell 返回的指针指向 map.tunnels() 内部，可直接还原下标。
      const std::uint8_t id = static_cast<std::uint8_t>(
        (spec - map.tunnels().data()) + 1);
      body_points[id - 1].push_back(geometry.cellCenter(sx, sy));

      const int y0 = std::max(0, sy - span);
      const int y1 = std::min(height - 1, sy + span);
      const int x0 = std::max(0, sx - span);
      const int x1 = std::min(width - 1, sx + span);
      for (int ny = y0; ny <= y1; ++ny) {
        for (int nx = x0; nx <= x1; ++nx) {
          const double dx = static_cast<double>(nx - sx) * geometry.resolution;
          const double dy = static_cast<double>(ny - sy) * geometry.resolution;
          const double dist_sq = dx * dx + dy * dy;
          if (dist_sq > margin_sq_m && !(nx == sx && ny == sy)) {
            continue;
          }
          const std::size_t index = geometry.index(nx, ny);
          if (static_cast<float>(dist_sq) >= best_sq[index]) {
            continue;
          }
          best_sq[index] = static_cast<float>(dist_sq);
          spec_index[index] = id;
        }
      }
    }
  }

  if (!any) {
    return result;
  }

  // —— 走廊轴（PCA）：每条隧道一条。退化（单格 / 共线秩亏）时退回方向场或
  // +x，保证 axes_ 恒有可用方向；half_len=0 表示轴不可信，查询端会保守地
  // 维持旧的一刀切行为而不是放大盲区。
  result.axes_.resize(map.tunnels().size());
  for (std::size_t k = 0; k < map.tunnels().size(); ++k) {
    const auto & pts = body_points[k];
    if (pts.empty()) {
      continue;
    }
    CorridorAxis axis;
    axis.spec_id = static_cast<std::uint8_t>(k + 1);
    for (const auto & p : pts) {
      axis.centroid += p;
    }
    axis.centroid /= static_cast<double>(pts.size());

    double cxx = 0.0;
    double cxy = 0.0, cyy = 0.0;
    for (const auto & p : pts) {
      const double dx = p.x() - axis.centroid.x();
      const double dy = p.y() - axis.centroid.y();
      cxx += dx * dx; cxy += dx * dy; cyy += dy * dy;
    }
    // 主特征向量：解析解。cxy≈0 说明轴向已与坐标轴对齐。
    Eigen::Vector2d dir(1.0, 0.0);
    if (std::abs(cxy) > 1e-12) {
      const double lambda = 0.5 * ((cxx + cyy) +
        std::sqrt((cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
      dir = Eigen::Vector2d(lambda - cyy, cxy);
    } else if (cxx >= cyy) {
      dir = Eigen::Vector2d(1.0, 0.0);
    } else {
      dir = Eigen::Vector2d(0.0, 1.0);
    }
    const double norm = dir.norm();
    if (norm < 1e-9) {
      const Eigen::Vector2d field_dir =
        map.directionAtCell(
          static_cast<int>(axis.centroid.x() / geometry.resolution + geometry.origin.x()),
          static_cast<int>(axis.centroid.y() / geometry.resolution + geometry.origin.y()));
      dir = field_dir.norm() > 1e-6 ? field_dir : Eigen::Vector2d(1.0, 0.0);
    } else {
      dir /= norm;
    }

    double half_len = 0.0;
    for (const auto & p : pts) {
      half_len = std::max(half_len,
        std::abs((p - axis.centroid).dot(dir)));
    }
    axis.dir = dir;
    axis.half_len = half_len + geometry.resolution * 0.5;  // 含半格半径
    result.axes_[k] = axis;
  }

  result.margin_m_ = std::max(0.0, margin_m);
  result.geometry_ = geometry;
  result.spec_index_ = std::move(spec_index);
  result.tunnels_ = map.tunnels();
  return result;
}

bool TunnelRegionGrid::pointOutsideCorridor(
  const double world_x, const double world_y) const noexcept
{
  if (axes_.empty()) {
    return false;  // 轴不可用：保守地维持旧一刀切语义，不扩大盲区也不缩小保护。
  }
  const auto cell = geometry_.containingCell(Eigen::Vector2d(world_x, world_y));
  if (!cell) {
    return false;
  }
  const std::uint8_t id = spec_index_[geometry_.index(cell->x(), cell->y())];
  if (id == 0 || static_cast<std::size_t>(id - 1) >= tunnels_.size()) {
    return false;
  }
  const CorridorAxis & axis = axes_[id - 1];
  if (axis.half_len <= 0.0) {
    return false;  // 单格退化：无法可靠定向，保守处理。
  }

  const Eigen::Vector2d delta(world_x - axis.centroid.x(),
                              world_y - axis.centroid.y());
  const double along = std::abs(delta.dot(axis.dir));
  // 垂直分量用叉积模长，对方向 ±u 不敏感。
  const double lateral = std::abs(delta.x() * axis.dir.y() - delta.y() * axis.dir.x());

  const TunnelSpec & spec = tunnels_[id - 1];
  const double half_width = spec.clear_width * 0.5 + margin_m_;
  const double reach = axis.half_len + spec.run_up;

  return (along > reach) || (lateral > half_width);
}

const TunnelSpec * TunnelRegionGrid::specNearPoint(
  const double world_x, const double world_y) const noexcept
{
  if (spec_index_.empty()) {
    return nullptr;
  }
  const auto cell = geometry_.containingCell(Eigen::Vector2d(world_x, world_y));
  if (!cell) {
    return nullptr;
  }
  const std::uint8_t id = spec_index_[geometry_.index(cell->x(), cell->y())];
  if (id == 0 || static_cast<std::size_t>(id - 1) >= tunnels_.size()) {
    return nullptr;
  }
  return &tunnels_[static_cast<std::size_t>(id - 1)];
}

bool TunnelRegionGrid::corridorFrameAtPoint(
  const double world_x, const double world_y, const double robot_radius,
  const double lateral_margin, CorridorFrame & out) const noexcept
{
  // 与 pointOutsideCorridor 同一套「找不到就保守返回 false」的语义：轴退化
  // （half_len<=0）、不在影响区、轴表为空，三种情况都给不了约束。
  if (axes_.empty()) {
    return false;
  }
  const auto cell = geometry_.containingCell(Eigen::Vector2d(world_x, world_y));
  if (!cell) {
    return false;
  }
  const std::uint8_t id = spec_index_[geometry_.index(cell->x(), cell->y())];
  if (id == 0 || static_cast<std::size_t>(id - 1) >= tunnels_.size()) {
    return false;
  }
  const CorridorAxis & axis = axes_[id - 1];
  if (axis.half_len <= 0.0) {
    return false;
  }

  const TunnelSpec & spec = tunnels_[id - 1];
  // 物理横向半宽：车体中心离轴线最多这么远，圆形车体才不擦壁。
  //
  // 钳到 0 而不是返回 false：RMUL 的真实隧道净宽 0.5 m、车体半径 0.25 m，
  // 相减恰好是 0 —— 这是「车体直径等于净宽、物理余量为零」的正常情形，
  // 正确的约束是压轴线（半宽 0 的走廊 = 轴线），而不是放弃约束。返回 false
  // 会让整个走廊特性在真实地图上静默失效（这就是最初的实现犯的错）。
  // 只有负值（净宽真的放不下车体，隧道不可通行）才没有意义，也一并钳到 0，
  // 由静态地图的壁面致命格去否决。
  const double half_width_inner =
    std::max(0.0, spec.clear_width * 0.5 - robot_radius) + std::max(0.0, lateral_margin);

  out.centroid = axis.centroid;
  out.dir = axis.dir;
  out.half_len = axis.half_len;
  out.half_width_inner = half_width_inner;
  // 与 pointOutsideCorridor 的横向半宽同式：影响区外沿。
  out.lateral_outer = spec.clear_width * 0.5 + margin_m_;
  return true;
}

std::vector<float> makeInflationRadiusLimit(
  const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map, double default_radius,
  double robot_radius, const TunnelRegionGrid & region)
{
  if (region.empty()) {
    // 没建影响区（margin 关掉或图里没隧道）时退回只认本体格的版本。
    return makeInflationRadiusLimit(grid, map, default_radius, robot_radius);
  }
  if (!map.valid() || map.tunnels().empty() || grid.data.empty() ||
    grid.info.resolution <= 0.0F)
  {
    return {};
  }

  std::vector<float> limits(grid.data.size(), static_cast<float>(default_radius));
  bool any_tunnel = false;

  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      double world_x = 0.0;
      double world_y = 0.0;
      mapToWorld(grid, x, y, world_x, world_y);
      const TunnelSpec * spec = region.specNearPoint(world_x, world_y);
      if (spec == nullptr) {
        continue;
      }
      any_tunnel = true;
      // 与本体格版本同一套语义：区内只留通道自身的横向余量，洞口不再被两侧墙的
      // 全量膨胀涂满。壁面格自己仍是致命的，压小半径不等于放开碰撞。
      const double clearance = std::max(0.0, spec->clear_width * 0.5 - robot_radius);
      limits[gridIndex(grid, x, y)] =
        static_cast<float>(std::min(default_radius, clearance));
    }
  }

  if (!any_tunnel) {
    return {};
  }
  return limits;
}

TunnelAxisGrid TunnelAxisGrid::build(
  const nav_msgs::msg::OccupancyGrid & grid, const SemanticMap & map)
{
  TunnelAxisGrid result;
  if (!map.valid() || grid.data.empty() || grid.info.resolution <= 0.0F) {
    return result;
  }

  std::vector<float> axis_x(grid.data.size(), 0.0F);
  std::vector<float> axis_y(grid.data.size(), 0.0F);
  bool any_tunnel = false;

  for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
      double world_x = 0.0;
      double world_y = 0.0;
      mapToWorld(grid, x, y, world_x, world_y);
      const auto cell = map.geometry().containingCell(Eigen::Vector2d(world_x, world_y));
      if (!cell) {
        continue;
      }
      // 只取本体格。膨胀圈的方向信息是弱的（模长 ≤ 0.9），拿它做硬性的轴向否决会
      // 在洞口外面就开始拦转向 —— 那是 MINCO 软代价的职责，不是 A* 的。
      if (!map.isTunnelBodyCell(cell->x(), cell->y())) {
        continue;
      }
      const Eigen::Vector2d axis = map.directionAtCell(cell->x(), cell->y());
      const double norm = axis.norm();
      if (norm < 1e-9) {
        continue;
      }
      any_tunnel = true;
      // 归一化后存：判据只关心方向，模长在本体格里是 ~1 但不精确（量化到 uint8）。
      const std::size_t index = gridIndex(grid, x, y);
      axis_x[index] = static_cast<float>(axis.x() / norm);
      axis_y[index] = static_cast<float>(axis.y() / norm);
    }
  }

  if (!any_tunnel) {
    return result;
  }
  result.axis_x_ = std::move(axis_x);
  result.axis_y_ = std::move(axis_y);
  return result;
}

bool TunnelAxisGrid::isTunnelCell(const std::size_t index) const noexcept
{
  if (index >= axis_x_.size()) {
    return false;
  }
  // 非隧道格两个分量都是 0（build 里只写隧道本体格），所以任一非零即隧道格。
  return axis_x_[index] != 0.0F || axis_y_[index] != 0.0F;
}

double TunnelAxisGrid::stepAlignment(
  const std::size_t from, const std::size_t to, const int step_x, const int step_y) const noexcept
{
  const bool from_tunnel = isTunnelCell(from);
  const bool to_tunnel = isTunnelCell(to);
  if (!from_tunnel && !to_tunnel) {
    return 1.0;
  }

  const double step_norm = std::hypot(static_cast<double>(step_x), static_cast<double>(step_y));
  if (step_norm < 1e-9) {
    return 1.0;
  }

  // 取在洞内那一端的轴线。隧道是直的，整条洞共用一条轴线，所以两端都在洞内时两者
  // 相同 —— 这里没有需要挑的东西。只有一端在洞内时（进洞/出洞的那一步）必须用洞内
  // 那端：否则洞口那一格不受约束，斜着切进去代价为 0。
  //
  // 将来若出现弯洞，这个取法在拐点上会依赖挑哪端，届时要改成取两端的较小值。
  const std::size_t axis_index = to_tunnel ? to : from;
  const double dot =
    (static_cast<double>(axis_x_[axis_index]) * static_cast<double>(step_x) +
    static_cast<double>(axis_y_[axis_index]) * static_cast<double>(step_y)) / step_norm;
  // 隧道双向，取绝对值：正着进和倒着进都合法。
  return std::clamp(std::abs(dot), 0.0, 1.0);
}

}  // namespace navigation2
