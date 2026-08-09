#ifndef CPP_LIDAR_FILTER__RADIAL_FILTER_HPP_
#define CPP_LIDAR_FILTER__RADIAL_FILTER_HPP_

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/types.h>

namespace cpp_lidar_filter
{

bool collectHorizontalRangeIndices(
  const pcl::PCLPointCloud2 & cloud, double range,
  const Eigen::Affine3d & target_from_cloud, pcl::Indices & indices);

}  // namespace cpp_lidar_filter

#endif  // CPP_LIDAR_FILTER__RADIAL_FILTER_HPP_
