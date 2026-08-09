#include "cpp_lidar_filter/radial_filter.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <pcl/common/io.h>

namespace cpp_lidar_filter
{
namespace
{

std::size_t fieldSize(const pcl::PCLPointField & field)
{
  switch (field.datatype) {
    case pcl::PCLPointField::FLOAT32:
      return sizeof(float);
    case pcl::PCLPointField::FLOAT64:
      return sizeof(double);
    default:
      return 0;
  }
}

bool readCoordinate(
  const std::uint8_t * point_data, const pcl::PCLPointField & field, double & value)
{
  if (field.datatype == pcl::PCLPointField::FLOAT32) {
    float coordinate;
    std::memcpy(&coordinate, point_data + field.offset, sizeof(coordinate));
    value = coordinate;
    return true;
  }
  if (field.datatype == pcl::PCLPointField::FLOAT64) {
    std::memcpy(&value, point_data + field.offset, sizeof(value));
    return true;
  }
  return false;
}

}  // namespace

bool collectHorizontalRangeIndices(
  const pcl::PCLPointCloud2 & cloud, double range,
  const Eigen::Affine3d & target_from_cloud, pcl::Indices & indices)
{
  indices.clear();
  if (!std::isfinite(range) || range <= 0.0 || cloud.is_bigendian ||
    !target_from_cloud.matrix().allFinite())
  {
    return false;
  }

  const int x_index = pcl::getFieldIndex(cloud, "x");
  const int y_index = pcl::getFieldIndex(cloud, "y");
  const int z_index = pcl::getFieldIndex(cloud, "z");
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return false;
  }

  const auto & x_field = cloud.fields[static_cast<std::size_t>(x_index)];
  const auto & y_field = cloud.fields[static_cast<std::size_t>(y_index)];
  const auto & z_field = cloud.fields[static_cast<std::size_t>(z_index)];
  const std::size_t x_size = fieldSize(x_field);
  const std::size_t y_size = fieldSize(y_field);
  const std::size_t z_size = fieldSize(z_field);
  if (x_field.count != 1 || y_field.count != 1 || z_field.count != 1 ||
    x_size == 0 || y_size == 0 || z_size == 0 ||
    x_field.offset + x_size > cloud.point_step ||
    y_field.offset + y_size > cloud.point_step ||
    z_field.offset + z_size > cloud.point_step)
  {
    return false;
  }

  const std::size_t packed_row_step =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.point_step);
  if (cloud.row_step != packed_row_step ||
    static_cast<std::size_t>(cloud.row_step) * cloud.height > cloud.data.size())
  {
    return false;
  }

  const std::size_t point_count =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (point_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  indices.reserve(point_count);

  const double range_squared = range * range;
  for (std::size_t row = 0; row < cloud.height; ++row) {
    const std::size_t row_offset = row * static_cast<std::size_t>(cloud.row_step);
    for (std::size_t column = 0; column < cloud.width; ++column) {
      const std::size_t point_offset =
        row_offset + column * static_cast<std::size_t>(cloud.point_step);
      if (point_offset > cloud.data.size() ||
        cloud.point_step > cloud.data.size() - point_offset)
      {
        indices.clear();
        return false;
      }

      const std::uint8_t * point_data = cloud.data.data() + point_offset;
      double x;
      double y;
      double z;
      if (!readCoordinate(point_data, x_field, x) ||
        !readCoordinate(point_data, y_field, y) ||
        !readCoordinate(point_data, z_field, z) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        continue;
      }

      const double target_x =
        target_from_cloud(0, 0) * x + target_from_cloud(0, 1) * y +
        target_from_cloud(0, 2) * z + target_from_cloud(0, 3);
      const double target_y =
        target_from_cloud(1, 0) * x + target_from_cloud(1, 1) * y +
        target_from_cloud(1, 2) * z + target_from_cloud(1, 3);
      if (!std::isfinite(target_x) || !std::isfinite(target_y)) {
        continue;
      }

      if (std::abs(target_x) <= range && std::abs(target_y) <= range &&
        target_x * target_x + target_y * target_y <= range_squared)
      {
        const std::size_t linear_index =
          row * static_cast<std::size_t>(cloud.width) + column;
        indices.push_back(static_cast<int>(linear_index));
      }
    }
  }

  return true;
}

}  // namespace cpp_lidar_filter
