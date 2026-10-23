#include "decision/center_patrol.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace decision
{

std::vector<Pose> generateZoneGrid(const CenterZone & zone, double spacing_m)
{
  std::vector<Pose> out;
  if (!zone.usable || zone.safe.size() < 3 || spacing_m <= 0.0) {
    return out;
  }

  double x0 = zone.safe.front().x;
  double x1 = x0;
  double y0 = zone.safe.front().y;
  double y1 = y0;
  for (const auto & v : zone.safe) {
    x0 = std::min(x0, v.x);
    x1 = std::max(x1, v.x);
    y0 = std::min(y0, v.y);
    y1 = std::max(y1, v.y);
  }

  const double span_x = x1 - x0;
  const double span_y = y1 - y0;
  if (span_x < spacing_m || span_y < spacing_m) {
    // 区域比一个格还小：至少给形心一个点。空 = 站着不动，那是最坏的结果。
    Pose c;
    for (const auto & v : zone.safe) {
      c.x += v.x / static_cast<double>(zone.safe.size());
      c.y += v.y / static_cast<double>(zone.safe.size());
    }
    if (zoneContains(zone, c)) {
      out.push_back(c);
    }
    return out;
  }

  // 格子数按"能放几个整格"取，然后**取格子中心** —— 见头文件里为什么不用边界刻度。
  const int nx = std::max(1, static_cast<int>(std::lround(span_x / spacing_m)));
  const int ny = std::max(1, static_cast<int>(std::lround(span_y / spacing_m)));
  const double dx = span_x / static_cast<double>(nx);
  const double dy = span_y / static_cast<double>(ny);

  for (int iy = 0; iy < ny; ++iy) {
    const bool left_to_right = (iy % 2 == 0);
    for (int k = 0; k < nx; ++k) {
      const int ix = left_to_right ? k : (nx - 1 - k);
      Pose p;
      p.x = x0 + dx * (static_cast<double>(ix) + 0.5);
      p.y = y0 + dy * (static_cast<double>(iy) + 0.5);
      if (zoneContains(zone, p)) {
        out.push_back(p);
      }
    }
  }
  return out;
}

PatrolPath takePatrolPath(std::size_t pool_size, std::size_t cursor, int path_len)
{
  PatrolPath path;
  if (pool_size == 0 || path_len <= 0) {
    return path;
  }
  const std::size_t count =
    std::min(static_cast<std::size_t>(path_len), pool_size);
  const std::size_t start = cursor % pool_size;
  path.indices.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    path.indices.push_back((start + i) % pool_size);
  }
  path.cursor = (start + count) % pool_size;
  return path;
}

}  // namespace decision
