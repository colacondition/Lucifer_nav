// 单遍过滤，内核换成 NUC_infra 的并行体素算子。
//
// 对外接口/语义与旧实现完全一致（调用方与测试不用改）：
//   - 判定在 navigation 系，输出点仍在**输入系**；
//   - 体素格按输入系量化，每格保留扫描序最靠前的点；
//   - 布局非法返回 false，空点云返回 true。
//
// 与旧实现的唯一行为差异（有意为之）：旧实现用 64 位 XOR 哈希当体素 key，
// 存在系统性碰撞（y/z 镜像体素互撞；合成帧实测单帧丢 ~1% 点），
// 新实现用三轴精确打包 key，不再误合并。
#include "cpp_lidar_filter/single_pass_filter.hpp"

#include <cstring>
#include <limits>
#include <vector>

#include <nuc_infra/voxel_filter.hpp>

namespace cpp_lidar_filter
{
namespace
{

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & msg, const char * name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

bool toDesc(
  const sensor_msgs::msg::PointField & f, std::size_t point_step, nuc_infra::FieldDesc & out)
{
  if (f.count != 1) {
    return false;
  }
  if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    if (f.offset + sizeof(float) > point_step) {
      return false;
    }
    out.offset = f.offset;
    out.type = nuc_infra::FieldType::kFloat32;
    out.present = true;
    return true;
  }
  if (f.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    if (f.offset + sizeof(double) > point_step) {
      return false;
    }
    out.offset = f.offset;
    out.type = nuc_infra::FieldType::kFloat64;
    out.present = true;
    return true;
  }
  return false;
}

}  // namespace

bool filterSinglePass(
  const sensor_msgs::msg::PointCloud2 & input, const Eigen::Affine3d & navigation_from_cloud,
  const SinglePassParams & params, pcl::PointCloud<pcl::PointXYZI> & output)
{
  output.clear();
  output.points.clear();
  if (input.is_bigendian || !navigation_from_cloud.matrix().allFinite()) {
    return false;
  }

  const auto * fx = findField(input, "x");
  const auto * fy = findField(input, "y");
  const auto * fz = findField(input, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    return false;
  }
  const std::size_t point_step = input.point_step;
  if (point_step == 0) {
    return false;
  }

  nuc_infra::CloudView view;
  if (!toDesc(*fx, point_step, view.x) || !toDesc(*fy, point_step, view.y) ||
    !toDesc(*fz, point_step, view.z))
  {
    return false;
  }
  view.point_step = point_step;
  view.data = input.data.data();
  view.size = input.data.size();

  if (const auto * fi = findField(input, "intensity")) {
    toDesc(*fi, point_step, view.intensity);
  }

  const std::size_t width = static_cast<std::size_t>(input.width);
  const std::size_t height = static_cast<std::size_t>(input.height);
  const std::size_t packed_row = width * point_step;
  const std::size_t row_step = static_cast<std::size_t>(input.row_step);
  if (row_step < packed_row) {
    return false;
  }
  const std::size_t required = height == 0 ? 0 : (height - 1) * row_step + packed_row;
  if (required > input.data.size()) {
    return false;
  }
  const std::size_t count = width * height;
  if (count > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return false;
  }
  view.n_points = count;

  // 常见情况（height==1 且 row 紧排）直接零拷贝；带行 padding 的组织化云走
  // 一次打包拷贝（雷达链路不会走到，保留旧实现的行为）。
  std::vector<std::uint8_t> packed;
  if (height > 1 || row_step != packed_row) {
    packed.resize(count * point_step);
    for (std::size_t r = 0; r < height; ++r) {
      std::memcpy(
        packed.data() + r * packed_row, input.data.data() + r * row_step, packed_row);
    }
    view.data = packed.data();
    view.size = packed.size();
    view.n_points = count;
  }

  nuc_infra::RigidTransformf tf;
  const auto & m = navigation_from_cloud.matrix();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      tf.r[i * 3 + j] = static_cast<float>(m(i, j));
    }
    tf.t[i] = static_cast<float>(m(i, 3));
  }

  nuc_infra::VoxelFilterParams np;
  np.range = static_cast<float>(params.range);
  np.leaf = static_cast<float>(params.leaf);
  np.body_min[0] = static_cast<float>(params.body_min_x);
  np.body_max[0] = static_cast<float>(params.body_max_x);
  np.body_min[1] = static_cast<float>(params.body_min_y);
  np.body_max[1] = static_cast<float>(params.body_max_y);
  np.body_min[2] = static_cast<float>(params.body_min_z);
  np.body_max[2] = static_cast<float>(params.body_max_z);
  np.negative = params.negative;

  static thread_local std::vector<nuc_infra::PointXYZI> scratch;
  if (!nuc_infra::voxelFilter(view, tf, np, scratch)) {
    return false;
  }

  output.points.resize(scratch.size());
  for (std::size_t i = 0; i < scratch.size(); ++i) {
    output.points[i] = pcl::PointXYZI(
      scratch[i].x, scratch[i].y, scratch[i].z, scratch[i].intensity);
  }
  output.width = static_cast<std::uint32_t>(output.points.size());
  output.height = 1;
  output.is_dense = true;
  return true;
}

}  // namespace cpp_lidar_filter
