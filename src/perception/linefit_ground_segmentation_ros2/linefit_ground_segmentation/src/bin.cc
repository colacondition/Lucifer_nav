#include "ground_segmentation/bin.h"

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
  has_point_ = true;
  if (z < min_z) {
    min_z = z;
    min_z_range = d;
  }
}

Bin::MinZPoint Bin::getMinZPoint() {
  MinZPoint point;

  if (has_point_) {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}

void Bin::reset() {
  has_point_ = false;
  min_z = std::numeric_limits<double>::max();
  min_z_range = 0.0;
}
