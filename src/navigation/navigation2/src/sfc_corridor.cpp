#include "sfc_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation2
{
namespace
{

// FNV-1a，与 global_planner_node 的 clearanceFieldHash 同一套约定：只比内容，
// 不比 header 时间戳 —— 定频重发的同一张图必须命中缓存。
std::uint64_t fnv1a(const void * data, std::size_t bytes)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const auto * bytes_ptr = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < bytes; ++i) {
    hash ^= bytes_ptr[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace

void SfcCorridor::updateGrid(
  const nav_msgs::msg::OccupancyGrid & grid, const SfcCorridorParams & params)
{
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  // 参数在位图之前校验：max_range 非法时整个生成器保持未就绪（见头注释第 3 点），
  // 不要带着半套状态继续跑。
  if (width <= 0 || height <= 0 || grid.info.resolution <= 0.0F ||
    grid.data.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) ||
    !std::isfinite(params.max_range) || params.max_range <= 0.0)
  {
    width_ = 0;
    height_ = 0;
    has_content_ = false;
    return;
  }

  // 致命位图。复用缓冲：同尺寸更新零分配（热路径纪律，见头注释）。
  const std::size_t cells = grid.data.size();
  if (lethal_.size() != cells) {
    lethal_.assign(cells, 0U);
  }

  const std::int8_t * src = grid.data.data();
  const int threshold = params.obstacle_threshold;
  const bool unknown_lethal = params.unknown_is_lethal;
  std::size_t lethal_count = 0;
  for (std::size_t i = 0; i < cells; ++i) {
    const std::int8_t value = src[i];
    // ROS OccupancyGrid：-1 未知、0 空闲、100 占据。任何 >= threshold 的正值都是
    // 致命（膨胀层会给 1..99，只有致命格是 100）。
    bool is_lethal = false;
    if (value < 0) {
      is_lethal = unknown_lethal;
    } else {
      is_lethal = static_cast<int>(value) >= threshold;
    }
    lethal_[i] = is_lethal ? 1U : 0U;
    lethal_count += is_lethal ? 1U : 0U;
  }

  // 内容哈希：位图 + 影响判定的全部参数。任何一个变了都要重建积分图。
  const std::uint64_t hash =
    fnv1a(lethal_.data(), cells) ^
    fnv1a(&params.obstacle_threshold, sizeof(params.obstacle_threshold)) ^
    fnv1a(&params.unknown_is_lethal, sizeof(params.unknown_is_lethal)) ^
    fnv1a(&grid.info.resolution, sizeof(grid.info.resolution)) ^
    fnv1a(&grid.info.origin.position, sizeof(grid.info.origin.position)) ^
    (static_cast<std::uint64_t>(width) << 32) ^ static_cast<std::uint64_t>(height);

  params_ = params;
  const bool geometry_changed =
    !has_content_ || width_ != width || height_ != height ||
    resolution_ != static_cast<double>(grid.info.resolution) ||
    origin_x_ != static_cast<double>(grid.info.origin.position.x) ||
    origin_y_ != static_cast<double>(grid.info.origin.position.y);

  width_ = width;
  height_ = height;
  resolution_ = static_cast<double>(grid.info.resolution);
  origin_x_ = static_cast<double>(grid.info.origin.position.x);
  origin_y_ = static_cast<double>(grid.info.origin.position.y);

  if (geometry_changed || hash != content_hash_) {
    updateRaw(width, height, resolution_, origin_x_, origin_y_, lethal_, params_);
  }
  content_hash_ = hash;
  has_content_ = true;
}

void SfcCorridor::updateRaw(
  int width, int height, double resolution, double origin_x, double origin_y,
  const std::vector<std::uint8_t> & lethal, const SfcCorridorParams & params)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    lethal.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) ||
    !std::isfinite(params.max_range) || params.max_range <= 0.0)
  {
    // max_range 非法时保持未就绪而不是带病工作 —— 见头注释第 3 点。
    width_ = 0;
    height_ = 0;
    has_content_ = false;
    return;
  }

  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  params_ = params;

  lethal_ = lethal;

  // 积分图。prefix_[(y)*(w+1)+(x)] = [0,y) x [0,x) 的致命数。
  // (w+1)*(h+1) 首行/首列恒 0，免去查询时的边界分支。
  const std::size_t stride = static_cast<std::size_t>(width) + 1;
  prefix_.assign(stride * static_cast<std::size_t>(height + 1), 0);
  for (int y = 0; y < height; ++y) {
    std::int32_t row_running = 0;
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(width);
    const std::size_t out_row = stride * static_cast<std::size_t>(y + 1);
    const std::size_t prev_row = stride * static_cast<std::size_t>(y);
    for (int x = 0; x < width; ++x) {
      row_running += lethal_[row + static_cast<std::size_t>(x)];
      prefix_[out_row + static_cast<std::size_t>(x) + 1] =
        prefix_[prev_row + static_cast<std::size_t>(x) + 1] + row_running;
    }
  }
  content_hash_ = 0;
  has_content_ = true;
}

bool SfcCorridor::insideMap(double x, double y) const noexcept
{
  if (!ready()) {
    return false;
  }
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return cx >= 0 && cx < width_ && cy >= 0 && cy < height_;
}

bool SfcCorridor::pointLethal(double x, double y) const noexcept
{
  if (!insideMap(x, y)) {
    return false;
  }
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return lethal_[static_cast<std::size_t>(cy) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(cx)] != 0;
}

std::int64_t SfcCorridor::lethalCount(int x0, int y0, int x1, int y1) const noexcept
{
  // 调用方保证 0 <= x0 <= x1 < width_、0 <= y0 <= y1 < height_。
  const std::size_t stride = static_cast<std::size_t>(width_) + 1;
  const std::int64_t a = prefix_[static_cast<std::size_t>(y1 + 1) * stride +
    static_cast<std::size_t>(x1) + 1];
  const std::int64_t b = prefix_[static_cast<std::size_t>(y0) * stride +
    static_cast<std::size_t>(x1) + 1];
  const std::int64_t c = prefix_[static_cast<std::size_t>(y1 + 1) * stride +
    static_cast<std::size_t>(x0)];
  const std::int64_t d = prefix_[static_cast<std::size_t>(y0) * stride +
    static_cast<std::size_t>(x0)];
  return a - b - c + d;
}

double SfcCorridor::clearanceRadius(double x, double y) const noexcept
{
  if (!ready() || !insideMap(x, y)) {
    return -1.0;
  }
  const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

  // 中心格本身致命：连 0 半宽的正方形都被污染，没有可用空间。
  if (lethal_[static_cast<std::size_t>(cy) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(cx)] != 0)
  {
    return -1.0;
  }

  // 半宽的三个上界：参数上限、地图边界。致命格约束由二分里的区间和判定。
  //
  // 注意 range_cells 必须钳到「地图最大可能的半宽」：max_range 可以被配成很大的
  // 有限值（TDT 原实现默认 FLT_MAX），直接 static_cast<int>(max_range/resolution)
  // 会溢出成 UB。钳到 max(width_, height_) 之后任何有限 max_range 都安全 ——
  // border 本来就 <= 这个值，参数再大也只是「不受参数限制」。
  double range_cells_d = params_.max_range / resolution_;
  const double max_possible =
    static_cast<double>(std::max(width_, height_));
  if (!(range_cells_d <= max_possible)) {  // 同时挡住 NaN
    range_cells_d = max_possible;
  }
  const int range_cells = static_cast<int>(range_cells_d);
  const int border = std::min(std::min(cx, width_ - 1 - cx), std::min(cy, height_ - 1 - cy));

  // 二分最大无污染半宽。区间和为零 <=> 该正方形内无致命格。
  // 精确到格：结果就是格数，不做亚格插值 —— 半格的差别小于一格的分辨率，
  // 而且保守（宁可小一格也不越过真实的致命格边界）。
  //
  // 不变量：lo 可行（中心格无致命，故 lo=0 恒成立），hi 不可行（或就是上界且可行）。
  int lo = 0;
  int hi = std::min(range_cells, border);
  if (hi <= 0) {
    return 0.0;
  }
  if (lethalCount(cx - hi, cy - hi, cx + hi, cy + hi) == 0) {
    return static_cast<double>(hi) * resolution_;
  }
  // hi 不可行，lo=0 可行：收缩到相邻。
  while (hi - lo > 1) {
    const int mid = lo + (hi - lo) / 2;
    if (lethalCount(cx - mid, cy - mid, cx + mid, cy + mid) == 0) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return static_cast<double>(lo) * resolution_;
}

SfcBox SfcCorridor::usableBoxAt(double x, double y) const noexcept
{
  const double radius = clearanceRadius(x, y);
  SfcBox box;
  if (radius < 0.0) {
    return box;  // 退化
  }
  const double usable = radius - params_.robot_radius;
  if (usable <= 0.0) {
    return box;  // 车体放不下：退化
  }
  box.x0 = x - usable;
  box.y0 = y - usable;
  box.x1 = x + usable;
  box.y1 = y + usable;
  return box;
}

bool SfcCorridor::pathClear(
  const std::vector<Eigen::Vector2d> & samples, double robot_radius) const noexcept
{
  if (samples.empty()) {
    return true;
  }
  if (!ready()) {
    // 没有全局图就无法判定。返回 false 会让调用方走「无法校验」分支，
    // 与 MINCO 现有距离场未就绪时的语义一致，而不是静默放行。
    return false;
  }
  for (const auto & p : samples) {
    if (!insideMap(p.x(), p.y())) {
      return false;
    }
    if (pointLethal(p.x(), p.y())) {
      return false;
    }
    const double radius = clearanceRadius(p.x(), p.y());
    if (radius < 0.0 || radius < robot_radius) {
      return false;
    }
  }
  return true;
}

}  // namespace navigation2
