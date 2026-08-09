#include "rc_esdf.h"

#include <limits>

void RcEsdfMap::initialize(double width_m, double height_m, double resolution)
{
  width_m_ = width_m;
  height_m_ = height_m;
  resolution_ = resolution;
  grid_size_x_ = static_cast<int>(std::ceil(width_m_ / resolution_));
  grid_size_y_ = static_cast<int>(std::ceil(height_m_ / resolution_));
  // 让机器人中心落在地图中心。
  origin_x_ = -width_m_ / 2.0;
  origin_y_ = -height_m_ / 2.0;
  data_.assign(grid_size_x_ * grid_size_y_, 0.0f);
}

// 逐格对多边形所有边求最小距离。这里刻意保持暴力实现：
// 只在构造时调用一次（车身足迹固定），几十毫秒的一次性开销换取距离场
// 处处精确。曾经把远场格子用 resolution_*2 粗估过，结果整个远场的
// 距离都变成常数 0.04 m，esdfPathSafe 对任何远场激光点都判碰撞，
// 车在空地上也会被否决到进 StuckReverse。不要再「优化」这个函数。
void RcEsdfMap::generateFromPolygon(const std::vector<Eigen::Vector2d> & polygon)
{
  for (int y = 0; y < grid_size_y_; ++y) {
    for (int x = 0; x < grid_size_x_; ++x) {
      const double px = origin_x_ + (x + 0.5) * resolution_;
      const double py = origin_y_ + (y + 0.5) * resolution_;
      const Eigen::Vector2d p(px, py);

      double min_dist_sq = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < polygon.size(); ++i) {
        const double d_sq =
          pointToSegmentDistSq(p, polygon[i], polygon[(i + 1) % polygon.size()]);
        if (d_sq < min_dist_sq) {
          min_dist_sq = d_sq;
        }
      }
      const double min_dist = std::sqrt(min_dist_sq);
      data_[y * grid_size_x_ + x] =
        isPointInPolygon(p, polygon) ? static_cast<float>(-min_dist)
                                     : static_cast<float>(min_dist);
    }
  }
  std::cout << "[RC-ESDF] Map generated. Grid: "
            << grid_size_x_ << "x" << grid_size_y_ << std::endl;
}

bool RcEsdfMap::query(
  const Eigen::Vector2d & pos_body, double & dist, Eigen::Vector2d & grad) const
{
  double gx, gy;
  posToGrid(pos_body, gx, gy);

  const double u = gx - 0.5;
  const double v = gy - 0.5;

  if (u < 0 || u >= grid_size_x_ - 1 || v < 0 || v >= grid_size_y_ - 1) {
    dist = 0.0;
    grad.setZero();
    return false;
  }

  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  const double alpha = u - x0;
  const double beta = v - y0;

  const float v00 = getRaw(x0,     y0);
  const float v10 = getRaw(x0 + 1, y0);
  const float v01 = getRaw(x0,     y0 + 1);
  const float v11 = getRaw(x0 + 1, y0 + 1);

  // 双线性插值。
  dist = (1 - alpha) * (1 - beta) * v00 +
         alpha       * (1 - beta) * v10 +
         (1 - alpha) * beta       * v01 +
         alpha       * beta       * v11;

  // 解析梯度。
  const double d_alpha = (1 - beta) * (v10 - v00) + beta * (v11 - v01);
  const double d_beta  = (1 - alpha) * (v01 - v00) + alpha * (v11 - v10);
  grad.x() = d_alpha / resolution_;
  grad.y() = d_beta  / resolution_;

  return true;
}

double RcEsdfMap::pointToSegmentDistSq(
  const Eigen::Vector2d & p,
  const Eigen::Vector2d & v,
  const Eigen::Vector2d & w)
{
  const double l2 = (v - w).squaredNorm();
  if (l2 == 0.0) {
    return (p - v).squaredNorm();
  }
  const double t = std::max(0.0, std::min(1.0, (p - v).dot(w - v) / l2));
  return (p - (v + t * (w - v))).squaredNorm();
}

bool RcEsdfMap::isPointInPolygon(
  const Eigen::Vector2d & p,
  const std::vector<Eigen::Vector2d> & poly)
{
  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y() > p.y()) != (poly[j].y() > p.y())) &&
      (p.x() < (poly[j].x() - poly[i].x()) * (p.y() - poly[i].y()) /
               (poly[j].y() - poly[i].y()) + poly[i].x()))
    {
      inside = !inside;
    }
  }
  return inside;
}
