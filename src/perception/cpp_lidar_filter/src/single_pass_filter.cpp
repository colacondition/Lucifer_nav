#include "cpp_lidar_filter/single_pass_filter.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_set>

namespace cpp_lidar_filter
{
namespace
{

std::size_t fieldSize(std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      return sizeof(float);
    case sensor_msgs::msg::PointField::FLOAT64:
      return sizeof(double);
    default:
      return 0;
  }
}

bool readCoordinate(
  const std::uint8_t * point_data, const sensor_msgs::msg::PointField & field, double & value)
{
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    float coordinate;
    std::memcpy(&coordinate, point_data + field.offset, sizeof(coordinate));
    value = coordinate;
    return true;
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    std::memcpy(&value, point_data + field.offset, sizeof(value));
    return true;
  }
  return false;
}

// 体素格量化成 64 位 key（负坐标安全；无符号乘法回绕是定义良好的）。
std::uint64_t voxelKey(std::int64_t vx, std::int64_t vy, std::int64_t vz)
{
  const std::uint64_t ux = static_cast<std::uint64_t>(vx);
  const std::uint64_t uy = static_cast<std::uint64_t>(vy);
  const std::uint64_t uz = static_cast<std::uint64_t>(vz);
  return (ux * 0x9E3779B97F4A7C15ULL) ^
         (uy * 0xC2B2AE3D27D4EB4FULL) ^
         (uz * 0x165667B19E3779F9ULL);
}

}  // namespace

bool filterSinglePass(
  const sensor_msgs::msg::PointCloud2 & input,
  const Eigen::Affine3d & navigation_from_cloud,
  const SinglePassParams & params,
  pcl::PointCloud<pcl::PointXYZI> & output)
{
  output.clear();
  if (!std::isfinite(params.range) || params.range <= 0.0 ||
    !std::isfinite(params.leaf) || params.leaf <= 0.0 ||
    input.is_bigendian || !navigation_from_cloud.matrix().allFinite())
  {
    return false;
  }

  const auto field_index = [&input](const char * name) {
    for (std::size_t i = 0; i < input.fields.size(); ++i) {
      if (input.fields[i].name == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  const int x_index = field_index("x");
  const int y_index = field_index("y");
  const int z_index = field_index("z");
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return false;
  }
  const int intensity_index = field_index("intensity");

  const auto & x_field = input.fields[static_cast<std::size_t>(x_index)];
  const auto & y_field = input.fields[static_cast<std::size_t>(y_index)];
  const auto & z_field = input.fields[static_cast<std::size_t>(z_index)];
  const std::size_t x_size = fieldSize(x_field.datatype);
  const std::size_t y_size = fieldSize(y_field.datatype);
  const std::size_t z_size = fieldSize(z_field.datatype);
  if (x_field.count != 1 || y_field.count != 1 || z_field.count != 1 ||
    x_size == 0 || y_size == 0 || z_size == 0 ||
    x_field.offset + x_size > input.point_step ||
    y_field.offset + y_size > input.point_step ||
    z_field.offset + z_size > input.point_step)
  {
    return false;
  }
  bool has_intensity = false;
  std::size_t intensity_offset = 0;
  if (intensity_index >= 0) {
    const auto & intensity_field = input.fields[static_cast<std::size_t>(intensity_index)];
    if (intensity_field.count == 1 &&
      intensity_field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      intensity_field.offset + sizeof(float) <= input.point_step)
    {
      has_intensity = true;
      intensity_offset = intensity_field.offset;
    }
  }

  const std::size_t point_step = static_cast<std::size_t>(input.point_step);
  if (point_step == 0 || input.width == 0 || input.height == 0) {
    return false;
  }
  const std::size_t packed_row_step = static_cast<std::size_t>(input.width) * point_step;
  if (input.row_step != packed_row_step ||
    static_cast<std::size_t>(input.row_step) * input.height > input.data.size())
  {
    return false;
  }

  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);
  if (point_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return false;
  }

  const double range_squared = params.range * params.range;
  const double inv_leaf = 1.0 / params.leaf;

  // 节点成员复用：clear() 保留 capacity。seen 使用 thread_local，组件固定单回调线程，
  // 同尺寸帧只clear桶内容、不重新申请整张哈希表。
  output.reserve(point_count);
  thread_local std::unordered_set<std::uint64_t> seen;
  seen.clear();
  seen.reserve(point_count);

  // 旋转/平移分量拆开，避免逐点构造 Affine 乘法的开销。
  const auto & R = navigation_from_cloud.linear();
  const Eigen::Vector3d t = navigation_from_cloud.translation();

  for (std::size_t row = 0; row < input.height; ++row) {
    const std::size_t row_offset = row * static_cast<std::size_t>(input.row_step);
    for (std::size_t column = 0; column < input.width; ++column) {
      const std::size_t point_offset = row_offset + column * point_step;
      if (point_offset > input.data.size() ||
        point_step > input.data.size() - point_offset)
      {
        output.clear();
        return false;
      }

      const std::uint8_t * point_data = input.data.data() + point_offset;
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

      // 判定全部在 navigation 系（base_link）做：半径、车身 box 与 RViz marker
      // 同系同语义。
      const double tx = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + t.x();
      const double ty = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + t.y();
      const double tz = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + t.z();
      if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz)) {
        continue;
      }
      if (std::abs(tx) > params.range || std::abs(ty) > params.range ||
        tx * tx + ty * ty > range_squared)
      {
        continue;
      }
      const bool inside_box =
        tx >= params.body_min_x && tx <= params.body_max_x &&
        ty >= params.body_min_y && ty <= params.body_max_y &&
        tz >= params.body_min_z && tz <= params.body_max_z;
      if (inside_box == params.negative) {
        continue;  // negative=true 时挖掉 box 内；false 时只留 box 内
      }

      // 近似体素：按输入系坐标量化，每格保留首个点（ApproximateVoxelGrid 语义，
      // 对 0.06m 降采样足够，且无每格质心聚合的开销）。
      const std::uint64_t key = voxelKey(
        static_cast<std::int64_t>(std::floor(x * inv_leaf)),
        static_cast<std::int64_t>(std::floor(y * inv_leaf)),
        static_cast<std::int64_t>(std::floor(z * inv_leaf)));
      if (!seen.insert(key).second) {
        continue;
      }

      pcl::PointXYZI point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      if (has_intensity) {
        float intensity;
        std::memcpy(&intensity, point_data + intensity_offset, sizeof(intensity));
        point.intensity = std::isfinite(intensity) ? intensity : 0.0F;
      } else {
        point.intensity = 0.0F;
      }
      output.push_back(point);
    }
  }

  return true;
}

}  // namespace cpp_lidar_filter
