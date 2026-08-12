#include "semantic_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <msgpack.hpp>

namespace navigation2
{

namespace
{

constexpr double kTwoPi = 2.0 * M_PI;

Eigen::Vector2d decodeAxis(std::uint8_t encoded)
{
  const double angle = static_cast<double>(encoded) / 255.0 * kTwoPi;
  return {std::cos(angle), std::sin(angle)};
}

// 双线性采样模板。采样点对齐格心：q = (p - origin) / res - 0.5，于是格心处
// 权重恰好落在单个格子上。越界时钳到边界格并把该轴的权重梯度归零，让代价在
// 地图边缘保持连续（而不是突然掉到 0）。
struct BilinearStencil
{
  std::array<Eigen::Vector2i, 4> cells;
  std::array<double, 4> weights;
  std::array<Eigen::Vector2d, 4> weight_gradients;
};

BilinearStencil makeStencil(const GridGeometry & geometry, const Eigen::Vector2d & point)
{
  Eigen::Vector2d q = (point - geometry.origin) / geometry.resolution -
    Eigen::Vector2d::Constant(0.5);
  // 把几乎正好落在格心的点吸附过去，避免浮点除法的尾数噪声让权重在 1.0 和
  // 0.9999 之间抖动。
  for (int axis = 0; axis < 2; ++axis) {
    const double nearest = std::round(q(axis));
    if (std::abs(q(axis) - nearest) <= 1e-10) {
      q(axis) = nearest;
    }
  }

  const double unclamped_x = q.x();
  const double unclamped_y = q.y();
  q.x() = std::clamp(q.x(), 0.0, static_cast<double>(geometry.width - 1));
  q.y() = std::clamp(q.y(), 0.0, static_cast<double>(geometry.height - 1));

  const int x0 = static_cast<int>(std::floor(q.x()));
  const int y0 = static_cast<int>(std::floor(q.y()));
  const int x1 = std::min(x0 + 1, geometry.width - 1);
  const int y1 = std::min(y0 + 1, geometry.height - 1);
  const double tx = q.x() - static_cast<double>(x0);
  const double ty = q.y() - static_cast<double>(y0);
  const double inv_res = 1.0 / geometry.resolution;

  BilinearStencil stencil;
  stencil.cells = {{{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}}};
  stencil.weights = {{
    (1.0 - tx) * (1.0 - ty),
    tx * (1.0 - ty),
    (1.0 - tx) * ty,
    tx * ty,
  }};
  stencil.weight_gradients = {{
    {-(1.0 - ty) * inv_res, -(1.0 - tx) * inv_res},
    {(1.0 - ty) * inv_res, -tx * inv_res},
    {-ty * inv_res, (1.0 - tx) * inv_res},
    {ty * inv_res, tx * inv_res},
  }};

  if (unclamped_x < 0.0 || unclamped_x > static_cast<double>(geometry.width - 1)) {
    for (auto & gradient : stencil.weight_gradients) {
      gradient.x() = 0.0;
    }
  }
  if (unclamped_y < 0.0 || unclamped_y > static_cast<double>(geometry.height - 1)) {
    for (auto & gradient : stencil.weight_gradients) {
      gradient.y() = 0.0;
    }
  }
  return stencil;
}

void validateInflationParams(const InflationParams & params)
{
  if (!std::isfinite(params.resolution) || params.resolution <= 0.0) {
    throw std::invalid_argument("semantic map inflation resolution must be finite and positive");
  }
  if (!std::isfinite(params.full_cost_radius_m) || params.full_cost_radius_m < 0.0 ||
    !std::isfinite(params.cutoff_radius_m) ||
    params.cutoff_radius_m < params.full_cost_radius_m)
  {
    throw std::invalid_argument(
      "semantic map inflation requires 0 <= full_cost_radius_m <= cutoff_radius_m");
  }
  if (!std::isfinite(params.decay_rate_per_m) || params.decay_rate_per_m < 0.0) {
    throw std::invalid_argument("semantic map decay_rate_per_m must be finite and non-negative");
  }
  if (!std::isfinite(params.non_body_magnitude_cap) || params.non_body_magnitude_cap <= 0.0 ||
    params.non_body_magnitude_cap > kMaxInflatedMagnitude)
  {
    throw std::invalid_argument("semantic map non_body_magnitude_cap must be in (0, 0.9]");
  }
}

// 距离 → 衰减系数。满代价半径内为 1，之后指数衰减，超过截断半径为 0。
double decayAt(double distance_m, const InflationParams & params)
{
  if (distance_m > params.cutoff_radius_m) {
    return 0.0;
  }
  if (distance_m <= params.full_cost_radius_m) {
    return 1.0;
  }
  return std::exp(-params.decay_rate_per_m * (distance_m - params.full_cost_radius_m));
}

// 精确欧氏距离变换（Felzenszwalb & Huttenlocher 两遍抛物线下包络），返回每格到
// 最近种子格的距离（单位：格）。
//
// 这里不用 distance_field.cpp 的 Dijkstra 八邻域扩散：那个版本对 45° 之外的方向
// 系统性高估最多约 8%，而膨胀半径只有 6 格，误差直接落在代价梯度上。也不引
// OpenCV —— 本包目前不链接它，为一个距离变换拖进整个 imgproc 不值得。
std::vector<double> exactSquaredDistanceTransform(
  const std::vector<std::uint8_t> & seeds, const int width, const int height)
{
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  // 用有限大数而不是 inf 表示「无种子」：抛物线求交点要做减法和除法，inf 会出
  // NaN。取 1e12 使得开方后（1e6 格）远超任何真实地图尺寸，等价于无穷远。
  constexpr double kNoSeed = 1e12;
  std::vector<double> squared(cell_count, kNoSeed);

  // 一维下包络：对 f 求 d(q) = min_p (f(p) + (q - p)^2)。
  const auto transform1d = [](const std::vector<double> & f, std::vector<int> & v,
      std::vector<double> & z, std::vector<double> & out, const int n) {
      constexpr double kInf1d = std::numeric_limits<double>::infinity();
      int k = 0;
      v[0] = 0;
      z[0] = -kInf1d;
      z[1] = kInf1d;
      for (int q = 1; q < n; ++q) {
        const double fq = f[static_cast<std::size_t>(q)] + static_cast<double>(q) *
          static_cast<double>(q);
        while (true) {
          const double vk = static_cast<double>(v[static_cast<std::size_t>(k)]);
          const double fv = f[static_cast<std::size_t>(v[static_cast<std::size_t>(k)])] + vk * vk;
          const double s = (fq - fv) / (2.0 * static_cast<double>(q) - 2.0 * vk);
          if (s <= z[static_cast<std::size_t>(k)] && k > 0) {
            --k;
            continue;
          }
          ++k;
          v[static_cast<std::size_t>(k)] = q;
          z[static_cast<std::size_t>(k)] = s;
          z[static_cast<std::size_t>(k) + 1] = kInf1d;
          break;
        }
      }

      k = 0;
      for (int q = 0; q < n; ++q) {
        while (z[static_cast<std::size_t>(k) + 1] < static_cast<double>(q)) {
          ++k;
        }
        const int p = v[static_cast<std::size_t>(k)];
        const double delta = static_cast<double>(q - p);
        out[static_cast<std::size_t>(q)] = delta * delta + f[static_cast<std::size_t>(p)];
      }
    };

  const int max_dim = std::max(width, height);
  std::vector<double> f(static_cast<std::size_t>(max_dim), 0.0);
  std::vector<double> out(static_cast<std::size_t>(max_dim), 0.0);
  std::vector<int> v(static_cast<std::size_t>(max_dim), 0);
  std::vector<double> z(static_cast<std::size_t>(max_dim) + 1, 0.0);

  // 先按列做，再按行做。
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      f[static_cast<std::size_t>(y)] = seeds[index] != 0 ? 0.0 : kNoSeed;
    }
    transform1d(f, v, z, out, height);
    for (int y = 0; y < height; ++y) {
      const std::size_t index = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
        static_cast<std::size_t>(x);
      squared[index] = out[static_cast<std::size_t>(y)];
    }
  }
  for (int y = 0; y < height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    for (int x = 0; x < width; ++x) {
      f[static_cast<std::size_t>(x)] = squared[row + static_cast<std::size_t>(x)];
    }
    transform1d(f, v, z, out, width);
    for (int x = 0; x < width; ++x) {
      squared[row + static_cast<std::size_t>(x)] = out[static_cast<std::size_t>(x)];
    }
  }

  for (double & value : squared) {
    value = std::sqrt(std::max(0.0, value));
  }
  return squared;
}

}  // namespace

const char * terrainLabelName(const std::uint8_t label)
{
  switch (static_cast<TerrainType>(label)) {
    case TerrainType::FLAT:
      return "flat";
    case TerrainType::OBSTACLE:
      return "obstacle";
    case TerrainType::TUNNEL:
      return "tunnel";
    case TerrainType::UNKNOWN:
      return "unknown";
    default:
      return "reserved";
  }
}

bool GridGeometry::valid() const noexcept
{
  return width > 0 && height > 0 && std::isfinite(resolution) && resolution > 0.0 &&
         origin.allFinite();
}

std::size_t GridGeometry::cellCount() const noexcept
{
  if (width <= 0 || height <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
}

bool GridGeometry::containsCell(const int x, const int y) const noexcept
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

bool GridGeometry::containsPoint(const Eigen::Vector2d & point) const noexcept
{
  if (!point.allFinite()) {
    return false;
  }
  const Eigen::Vector2d max = origin + resolution * Eigen::Vector2d(width, height);
  return point.x() >= origin.x() && point.y() >= origin.y() && point.x() < max.x() &&
         point.y() < max.y();
}

std::size_t GridGeometry::index(const int x, const int y) const noexcept
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(x);
}

std::optional<Eigen::Vector2i> GridGeometry::containingCell(const Eigen::Vector2d & point) const
noexcept
{
  if (!containsPoint(point)) {
    return std::nullopt;
  }
  const Eigen::Array2i cell = ((point - origin) / resolution).array().floor().cast<int>();
  return Eigen::Vector2i(std::clamp(cell.x(), 0, width - 1), std::clamp(cell.y(), 0, height - 1));
}

Eigen::Vector2d GridGeometry::cellCenter(const int x, const int y) const noexcept
{
  return origin + resolution * (Eigen::Vector2d(x, y) + Eigen::Vector2d::Constant(0.5));
}

bool GridGeometry::sameAs(const GridGeometry & other) const noexcept
{
  return width == other.width && height == other.height &&
         std::abs(resolution - other.resolution) <= 1e-9 &&
         (origin - other.origin).cwiseAbs().maxCoeff() <= 1e-9;
}

bool SemanticMapData::valid() const noexcept
{
  const std::size_t expected = geometry.cellCount();
  if (!geometry.valid() || expected == 0) {
    return false;
  }
  if (terrain.size() != expected || direction.size() != expected) {
    return false;
  }
  if (!tunnel_ids.empty() && tunnel_ids.size() != expected) {
    return false;
  }
  return true;
}

std::uint8_t SemanticMap::terrainAtCell(const int x, const int y) const noexcept
{
  if (!geometry_.containsCell(x, y)) {
    return static_cast<std::uint8_t>(TerrainType::OBSTACLE);
  }
  return terrain_[geometry_.index(x, y)];
}

std::uint8_t SemanticMap::costAtCell(const int x, const int y) const noexcept
{
  if (!geometry_.containsCell(x, y)) {
    return 255;
  }
  return cost_[geometry_.index(x, y)];
}

Eigen::Vector2d SemanticMap::directionAtCell(const int x, const int y) const noexcept
{
  if (!geometry_.containsCell(x, y)) {
    return Eigen::Vector2d::Zero();
  }
  return direction_[geometry_.index(x, y)];
}

double SemanticMap::magnitudeAtCell(const int x, const int y) const noexcept
{
  return directionAtCell(x, y).norm();
}

bool SemanticMap::isTerrainBodyCell(const int x, const int y) const noexcept
{
  return magnitudeAtCell(x, y) > kTerrainBodyMagnitudeThreshold;
}

bool SemanticMap::isTunnelBodyCell(const int x, const int y) const noexcept
{
  return terrainAtCell(x, y) == static_cast<std::uint8_t>(TerrainType::TUNNEL) &&
         isTerrainBodyCell(x, y);
}

const TunnelSpec * SemanticMap::tunnelSpecAtCell(const int x, const int y) const noexcept
{
  if (tunnels_.empty() || !isTunnelBodyCell(x, y)) {
    return nullptr;
  }
  if (tunnel_ids_.empty()) {
    return &tunnels_.front();
  }
  const std::uint8_t id = tunnel_ids_[geometry_.index(x, y)];
  if (id == 0 || static_cast<std::size_t>(id - 1) >= tunnels_.size()) {
    return &tunnels_.front();
  }
  return &tunnels_[static_cast<std::size_t>(id - 1)];
}

SemanticMap::CostSample SemanticMap::sampleCost(const Eigen::Vector2d & point) const noexcept
{
  CostSample sample;
  if (!valid() || !point.allFinite()) {
    return sample;
  }
  const BilinearStencil stencil = makeStencil(geometry_, point);
  for (std::size_t i = 0; i < stencil.cells.size(); ++i) {
    const double raw = static_cast<double>(costAtCell(stencil.cells[i].x(), stencil.cells[i].y()));
    sample.value += stencil.weights[i] * raw;
    sample.gradient += stencil.weight_gradients[i] * raw;
  }
  return sample;
}

SemanticMap::DirectionSample SemanticMap::sampleDirection(const Eigen::Vector2d & point) const
noexcept
{
  DirectionSample sample;
  if (!valid() || !point.allFinite()) {
    return sample;
  }
  const BilinearStencil stencil = makeStencil(geometry_, point);
  for (std::size_t i = 0; i < stencil.cells.size(); ++i) {
    const Eigen::Vector2d raw = directionAtCell(stencil.cells[i].x(), stencil.cells[i].y());
    sample.value += stencil.weights[i] * raw;
    sample.jacobian += raw * stencil.weight_gradients[i].transpose();
  }
  return sample;
}

SemanticMap::LabelWeights SemanticMap::sampleLabelWeights(const Eigen::Vector2d & point) const
noexcept
{
  LabelWeights result;
  if (!valid() || !point.allFinite()) {
    return result;
  }
  const BilinearStencil stencil = makeStencil(geometry_, point);
  for (std::size_t i = 0; i < stencil.cells.size(); ++i) {
    const std::size_t label = terrainAtCell(stencil.cells[i].x(), stencil.cells[i].y());
    if (label >= kTerrainLabelCount) {
      continue;
    }
    result.weights[label] += stencil.weights[i];
    result.dweights[label] += stencil.weight_gradients[i];
  }
  return result;
}

SemanticMap SemanticMap::inflate(const SemanticMapData & data, const InflationParams & params)
{
  if (!data.valid()) {
    throw std::invalid_argument("semantic map data is not self-consistent");
  }
  validateInflationParams(params);
  if (std::abs(params.resolution - data.geometry.resolution) > 1e-9) {
    throw std::invalid_argument("inflation resolution does not match map resolution");
  }

  const int width = data.geometry.width;
  const int height = data.geometry.height;
  const std::size_t cell_count = data.geometry.cellCount();

  SemanticMap map;
  map.geometry_ = data.geometry;
  map.terrain_ = data.terrain;
  map.tunnels_ = data.tunnels;
  map.tunnel_ids_ = data.tunnel_ids;
  map.cost_.assign(cell_count, 0);
  map.direction_.assign(cell_count, Eigen::Vector2d::Zero());

  // ---- 代价场 ----
  // 障碍格是种子，其余格按到最近障碍的欧氏距离指数衰减。UNKNOWN 不作种子：它
  // 只是没扫到，不是有东西，惩罚交给 A* 的 unknown_cost。
  std::vector<std::uint8_t> obstacle_seeds(cell_count, 0);
  for (std::size_t index = 0; index < cell_count; ++index) {
    if (data.terrain[index] == static_cast<std::uint8_t>(TerrainType::OBSTACLE)) {
      obstacle_seeds[index] = 1;
    }
  }
  const std::vector<double> obstacle_distance_px =
    exactSquaredDistanceTransform(obstacle_seeds, width, height);
  for (std::size_t index = 0; index < cell_count; ++index) {
    if (obstacle_seeds[index] != 0) {
      map.cost_[index] = 255;
      continue;
    }
    const double distance_m = obstacle_distance_px[index] * params.resolution;
    const double decay = decayAt(distance_m, params);
    map.cost_[index] = static_cast<std::uint8_t>(
      std::clamp(std::round(255.0 * decay), 0.0, 255.0));
  }

  // ---- 方向场 ----
  // 隧道本体格：模长 1.0，方向就是标注的轴线。
  std::vector<std::uint8_t> tunnel_seeds(cell_count, 0);
  bool has_tunnel = false;
  for (std::size_t index = 0; index < cell_count; ++index) {
    if (data.terrain[index] != static_cast<std::uint8_t>(TerrainType::TUNNEL)) {
      continue;
    }
    tunnel_seeds[index] = 1;
    has_tunnel = true;
    map.direction_[index] = decodeAxis(data.direction[index]);
  }
  if (!has_tunnel) {
    return map;
  }

  // 膨胀圈：按到最近隧道格的距离衰减，模长封顶在 non_body_magnitude_cap，方向取
  // 邻域内隧道轴线的加权平均。
  //
  // 与参考工程的关键差异：轴线无向，直接累加 v 会让 0° 和 180° 抵消成零向量。
  // 改累加二阶矩 (cos2θ, sin2θ) —— 倍角把 ±v 映射到同一点，平均后再折半，
  // 得到的是「主轴方向」而不是「平均向量」。
  const int radius = static_cast<int>(std::ceil(params.cutoff_radius_m / params.resolution));
  const std::vector<double> tunnel_distance_px =
    exactSquaredDistanceTransform(tunnel_seeds, width, height);

  std::vector<double> sum_cos2(cell_count, 0.0);
  std::vector<double> sum_sin2(cell_count, 0.0);
  for (int sy = 0; sy < height; ++sy) {
    for (int sx = 0; sx < width; ++sx) {
      const std::size_t source = data.geometry.index(sx, sy);
      if (tunnel_seeds[source] == 0) {
        continue;
      }
      const Eigen::Vector2d axis = map.direction_[source];
      const double double_angle = 2.0 * std::atan2(axis.y(), axis.x());
      const double source_cos2 = std::cos(double_angle);
      const double source_sin2 = std::sin(double_angle);

      const int y0 = std::max(0, sy - radius);
      const int y1 = std::min(height, sy + radius + 1);
      const int x0 = std::max(0, sx - radius);
      const int x1 = std::min(width, sx + radius + 1);
      for (int ny = y0; ny < y1; ++ny) {
        for (int nx = x0; nx < x1; ++nx) {
          const double distance_m =
            std::hypot(static_cast<double>(nx - sx), static_cast<double>(ny - sy)) *
            params.resolution;
          if (distance_m > params.cutoff_radius_m) {
            continue;
          }
          const double weight = decayAt(distance_m, params);
          const std::size_t index = data.geometry.index(nx, ny);
          sum_cos2[index] += source_cos2 * weight;
          sum_sin2[index] += source_sin2 * weight;
        }
      }
    }
  }

  for (std::size_t index = 0; index < cell_count; ++index) {
    if (tunnel_seeds[index] != 0) {
      continue;
    }
    // 障碍格不带方向：车体不可能在那儿，给它方向只会污染采样模板。
    if (data.terrain[index] == static_cast<std::uint8_t>(TerrainType::OBSTACLE)) {
      continue;
    }
    const double distance_m = tunnel_distance_px[index] * params.resolution;
    if (distance_m > params.cutoff_radius_m) {
      continue;
    }
    const double magnitude = std::min(decayAt(distance_m, params), params.non_body_magnitude_cap);
    if (magnitude <= 0.0) {
      continue;
    }
    const double moment = std::hypot(sum_cos2[index], sum_sin2[index]);
    if (moment < 1e-12) {
      // 邻域里的轴线两两垂直，主轴无定义。留零向量，下游按「无约束」处理。
      continue;
    }
    const double angle = 0.5 * std::atan2(sum_sin2[index], sum_cos2[index]);
    map.direction_[index] = magnitude * Eigen::Vector2d(std::cos(angle), std::sin(angle));
  }

  return map;
}

SemanticMap SemanticMap::fromChannels(
  const GridGeometry & geometry, std::vector<std::uint8_t> terrain,
  const std::vector<std::uint8_t> & direction_angle,
  const std::vector<std::uint8_t> & direction_magnitude, std::vector<std::uint8_t> cost,
  std::vector<TunnelSpec> tunnels, std::vector<std::uint8_t> tunnel_ids)
{
  const std::size_t cell_count = geometry.cellCount();
  if (!geometry.valid() || cell_count == 0) {
    throw std::runtime_error("semantic map channels come with invalid geometry");
  }
  // 抛而不是返回空图：通道长度对不上意味着发布端和接收端对栅格的理解不一致，此时
  // 每个格号都可能错位。静默返回空图会让消费端以为「这张图没有隧道」照常跑，顶板
  // 照旧封住洞口，而日志里什么都没有。
  const auto require = [cell_count](const char * name, std::size_t size) {
      if (size != cell_count) {
        throw std::runtime_error(
          std::string("semantic map channel '") + name + "' has " + std::to_string(size) +
          " entries, expected " + std::to_string(cell_count));
      }
    };
  require("terrain", terrain.size());
  require("direction_angle", direction_angle.size());
  require("direction_magnitude", direction_magnitude.size());
  require("cost", cost.size());
  // tunnel_ids 允许为空 —— 那表示所有隧道格共用 tunnels[0]。
  if (!tunnel_ids.empty()) {
    require("tunnel_ids", tunnel_ids.size());
  }

  SemanticMap map;
  map.geometry_ = geometry;
  map.terrain_ = std::move(terrain);
  map.cost_ = std::move(cost);
  map.tunnels_ = std::move(tunnels);
  map.tunnel_ids_ = std::move(tunnel_ids);
  map.direction_.resize(cell_count);
  for (std::size_t index = 0; index < cell_count; ++index) {
    const double magnitude = static_cast<double>(direction_magnitude[index]) / 255.0;
    if (magnitude > 0.0) {
      map.direction_[index] = magnitude * decodeAxis(direction_angle[index]);
    } else {
      map.direction_[index] = Eigen::Vector2d::Zero();
    }
  }
  return map;
}

double SemanticMap::axisAlignment(
  const Eigen::Vector2d & point, const Eigen::Vector2d & heading) const noexcept
{
  const double heading_norm = heading.norm();
  if (heading_norm < 1e-9) {
    return 1.0;
  }
  const Eigen::Vector2d axis = sampleDirection(point).value;
  const double axis_norm = axis.norm();
  if (axis_norm < 1e-9) {
    return 1.0;
  }
  // 隧道双向：取绝对值，正进和倒进等价。
  return std::clamp(std::abs(axis.dot(heading)) / (axis_norm * heading_norm), 0.0, 1.0);
}

namespace
{

const msgpack::object * findKey(const msgpack::object & root, const std::string & key)
{
  if (root.type != msgpack::type::MAP) {
    return nullptr;
  }
  for (std::uint32_t i = 0; i < root.via.map.size; ++i) {
    const msgpack::object & candidate = root.via.map.ptr[i].key;
    if (candidate.type != msgpack::type::STR) {
      continue;
    }
    if (key.size() == candidate.via.str.size &&
      std::equal(key.begin(), key.end(), candidate.via.str.ptr))
    {
      return &root.via.map.ptr[i].val;
    }
  }
  return nullptr;
}

const msgpack::object & requireKey(const msgpack::object & root, const std::string & key)
{
  const msgpack::object * value = findKey(root, key);
  if (value == nullptr) {
    throw std::runtime_error("semantic map is missing required field '" + key + "'");
  }
  return *value;
}

double asDouble(const msgpack::object & object, const std::string & field)
{
  switch (object.type) {
    case msgpack::type::FLOAT32:
    case msgpack::type::FLOAT64:
      return object.via.f64;
    case msgpack::type::POSITIVE_INTEGER:
      return static_cast<double>(object.via.u64);
    case msgpack::type::NEGATIVE_INTEGER:
      return static_cast<double>(object.via.i64);
    default:
      throw std::runtime_error("semantic map field '" + field + "' is not a number");
  }
}

int asInt(const msgpack::object & object, const std::string & field)
{
  const double value = asDouble(object, field);
  if (!std::isfinite(value) || value < 0.0 ||
    value > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::runtime_error("semantic map field '" + field + "' is not a valid size");
  }
  return static_cast<int>(value);
}

// 字节通道。Python 侧 bytes 会打成 BIN，但历史上 msgpack 也把 bytes 打成 STR
// （raw），两种都收；整数数组也收，方便手写测试数据。
std::vector<std::uint8_t> asByteChannel(
  const msgpack::object & object, const std::string & field, const std::size_t expected)
{
  std::vector<std::uint8_t> bytes;
  if (object.type == msgpack::type::BIN) {
    bytes.assign(object.via.bin.ptr, object.via.bin.ptr + object.via.bin.size);
  } else if (object.type == msgpack::type::STR) {
    bytes.assign(object.via.str.ptr, object.via.str.ptr + object.via.str.size);
  } else if (object.type == msgpack::type::ARRAY) {
    bytes.reserve(object.via.array.size);
    for (std::uint32_t i = 0; i < object.via.array.size; ++i) {
      const double value = asDouble(object.via.array.ptr[i], field);
      if (value < 0.0 || value > 255.0) {
        throw std::runtime_error("semantic map field '" + field + "' has a value outside 0~255");
      }
      bytes.push_back(static_cast<std::uint8_t>(std::lround(value)));
    }
  } else {
    throw std::runtime_error("semantic map field '" + field + "' is not a byte channel");
  }

  if (bytes.size() != expected) {
    throw std::runtime_error(
      "semantic map field '" + field + "' has " + std::to_string(bytes.size()) +
      " cells but the grid has " + std::to_string(expected));
  }
  return bytes;
}

}  // namespace

SemanticMapData loadSemanticMap(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open semantic map: " + path);
  }
  const std::string buffer(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  if (buffer.empty()) {
    throw std::runtime_error("semantic map is empty: " + path);
  }

  msgpack::object_handle handle;
  try {
    handle = msgpack::unpack(buffer.data(), buffer.size());
  } catch (const msgpack::parse_error & ex) {
    throw std::runtime_error("semantic map is not valid msgpack: " + path + ": " + ex.what());
  }
  const msgpack::object root = handle.get();
  if (root.type != msgpack::type::MAP) {
    throw std::runtime_error("semantic map root is not a map: " + path);
  }

  SemanticMapData data;
  data.geometry.width = asInt(requireKey(root, "width"), "width");
  data.geometry.height = asInt(requireKey(root, "height"), "height");
  data.geometry.resolution = asDouble(requireKey(root, "resolution"), "resolution");

  // origin 是本工程相对参考工程新增的字段，缺省按 0 处理 —— 参考工程导出的图就
  // 是原点在栅格角点，那种图直接读也应该对。
  if (const msgpack::object * origin = findKey(root, "origin")) {
    if (origin->type != msgpack::type::ARRAY || origin->via.array.size < 2) {
      throw std::runtime_error("semantic map 'origin' must be an array of at least 2 numbers");
    }
    data.geometry.origin.x() = asDouble(origin->via.array.ptr[0], "origin[0]");
    data.geometry.origin.y() = asDouble(origin->via.array.ptr[1], "origin[1]");
  }
  if (!data.geometry.valid()) {
    throw std::runtime_error("semantic map geometry is invalid: " + path);
  }

  const std::size_t cell_count = data.geometry.cellCount();
  data.terrain = asByteChannel(requireKey(root, "terrain"), "terrain", cell_count);
  data.direction = asByteChannel(requireKey(root, "direction"), "direction", cell_count);
  if (const msgpack::object * ids = findKey(root, "tunnel_ids")) {
    data.tunnel_ids = asByteChannel(*ids, "tunnel_ids", cell_count);
  }

  if (const msgpack::object * tunnels = findKey(root, "tunnels")) {
    if (tunnels->type != msgpack::type::ARRAY) {
      throw std::runtime_error("semantic map 'tunnels' must be an array");
    }
    data.tunnels.reserve(tunnels->via.array.size);
    for (std::uint32_t i = 0; i < tunnels->via.array.size; ++i) {
      const msgpack::object & entry = tunnels->via.array.ptr[i];
      if (entry.type != msgpack::type::MAP) {
        throw std::runtime_error("semantic map 'tunnels' entries must be maps");
      }
      const std::string label = "tunnels[" + std::to_string(i) + "]";
      TunnelSpec spec;
      spec.clear_height = asDouble(requireKey(entry, "clear_height"), label + ".clear_height");
      spec.clear_width = asDouble(requireKey(entry, "clear_width"), label + ".clear_width");
      if (const msgpack::object * run_up = findKey(entry, "run_up")) {
        spec.run_up = asDouble(*run_up, label + ".run_up");
      }
      if (const msgpack::object * v = findKey(entry, "velocity_min")) {
        spec.velocity_min = asDouble(*v, label + ".velocity_min");
      }
      if (const msgpack::object * v = findKey(entry, "velocity_max")) {
        spec.velocity_max = asDouble(*v, label + ".velocity_max");
      }
      if (!(spec.clear_height > 0.0) || !(spec.clear_width > 0.0)) {
        throw std::runtime_error(label + " must have positive clear_height and clear_width");
      }
      if (spec.run_up < 0.0 || spec.velocity_min < 0.0 || spec.velocity_max < 0.0 ||
        (spec.velocity_max > 0.0 && spec.velocity_min > spec.velocity_max))
      {
        throw std::runtime_error(label + " has an invalid run_up or velocity window");
      }
      data.tunnels.push_back(spec);
    }
  }

  // 自洽性检查。这些不变量在标注工具里就该保证，但地图文件是手工产物，宁可在
  // 加载时炸掉也不要让一张坏图静默地把车开进墙里。
  std::size_t tunnel_cells = 0;
  for (std::size_t index = 0; index < cell_count; ++index) {
    const std::uint8_t label = data.terrain[index];
    if (label >= kTerrainLabelCount) {
      throw std::runtime_error(
        "semantic map has an out-of-range terrain label " + std::to_string(label));
    }
    if (!isDirectionalLabel(label)) {
      // 非方向标签必须不带方向，否则膨胀时会当成轴线源传播出去。
      if (data.direction[index] != 0) {
        throw std::runtime_error(
          "semantic map cell " + std::to_string(index) + " is labelled " + terrainLabelName(label) +
          " but carries a direction");
      }
      continue;
    }
    ++tunnel_cells;
    if (!data.tunnel_ids.empty()) {
      const std::uint8_t id = data.tunnel_ids[index];
      if (id == 0 || static_cast<std::size_t>(id) > data.tunnels.size()) {
        throw std::runtime_error(
          "semantic map tunnel cell " + std::to_string(index) + " references tunnel id " +
          std::to_string(id) + " but only " + std::to_string(data.tunnels.size()) +
          " tunnels are declared");
      }
    }
  }
  if (tunnel_cells > 0 && data.tunnels.empty()) {
    throw std::runtime_error(
      "semantic map has tunnel cells but declares no tunnel specs; clear_height is required to "
      "decide whether the chassis fits");
  }

  return data;
}

}  // namespace navigation2
