#include "ground_segmentation/bin.h"

#include <cmath>
#include <limits>

Bin::Bin() : has_point_(false), min_z(std::numeric_limits<double>::max()), min_z_range(0.0) {}

Bin::Bin(const Bin& bin) : has_point_(false),
                           min_z(std::numeric_limits<double>::max()),
                           min_z_range(0.0) {}

void Bin::addPoint(const pcl::PointXYZ& point) {
  const double d = sqrt(point.x * point.x + point.y * point.y);
  addPoint(d, point.z);
}

void Bin::addPoint(const double& d, const double& z) {
  // NaN/Inf 点不能进 bin：z=NaN 时下面的 z < min_z 恒为假，旧实现却先把
  // has_point_ 置 true，于是这个 bin「有最小点」但 min_z 仍是 DBL_MAX，
  // getMinZPoint() 返回一个 z=1.8e308 的假点，会被当成地面线端点参与拟合，
  // 破坏整条 segment 的线（极端情况下把该方位的地面全判成障碍）。
  // 非有限点本来也不该参与分割，直接丢弃。
  if (!std::isfinite(d) || !std::isfinite(z)) {
    return;
  }
  has_point_ = true;
  // 快速路径：绝大多数点 z 不小于当前最小值，不进锁。只有更小的候选才需要
  // 在临界区内重新比较并原子地同时更新 (z, d)。
  if (z < min_z) {
    std::lock_guard<std::mutex> lock(update_mutex_);
    if (z < min_z) {
      min_z = z;
      min_z_range = d;
    }
  }
}

Bin::MinZPoint Bin::getMinZPoint() {
  MinZPoint point;

  std::lock_guard<std::mutex> lock(update_mutex_);
  if (has_point_) {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}

void Bin::reset() {
  std::lock_guard<std::mutex> lock(update_mutex_);
  has_point_ = false;
  min_z = std::numeric_limits<double>::max();
  min_z_range = 0.0;
}
