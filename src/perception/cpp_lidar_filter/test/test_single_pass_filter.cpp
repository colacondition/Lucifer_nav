#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cpp_lidar_filter/single_pass_filter.hpp"

namespace cpp_lidar_filter
{
namespace
{

// 构造紧凑的 x/y/z/intensity FLOAT32 点云。
sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<std::array<float, 4>> & points,
  bool with_intensity = true,
  bool with_xyz = true)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = with_xyz ? 16U : 4U;
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.row_step);

  if (with_xyz) {
    sensor_msgs::msg::PointField f;
    f.count = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.name = "x"; f.offset = 0; msg.fields.push_back(f);
    f.name = "y"; f.offset = 4; msg.fields.push_back(f);
    f.name = "z"; f.offset = 8; msg.fields.push_back(f);
    if (with_intensity) {
      f.name = "intensity"; f.offset = 12; msg.fields.push_back(f);
    }
  } else {
    sensor_msgs::msg::PointField f;
    f.count = 1;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.name = "intensity"; f.offset = 0; msg.fields.push_back(f);
  }

  std::size_t i = 0;
  for (const auto & p : points) {
    if (with_xyz) {
      std::memcpy(msg.data.data() + i * 16, p.data(), sizeof(float) * 4);
    } else {
      std::memcpy(msg.data.data() + i * 4, &p[3], sizeof(float));
    }
    ++i;
  }
  return msg;
}

SinglePassParams defaultParams()
{
  SinglePassParams p;
  p.range = 10.0;
  p.body_min_x = -0.4; p.body_max_x = 0.4;
  p.body_min_y = -0.3; p.body_max_y = 0.3;
  p.body_min_z = -0.1; p.body_max_z = 0.6;
  p.negative = true;
  p.leaf = 0.06;
  return p;
}

}  // namespace

TEST(SinglePassFilter, KeepsOnlyFinitePointsInsideOrOnRadiusBoundary)
{
  const auto input = makeCloud({
    {3.0F, 4.0F, 0.0F, 1.0F},   // 半径 5，保留
    {10.0F, 0.0F, 0.0F, 2.0F},  // 边界上，保留
    {8.0F, 8.0F, 0.0F, 3.0F},   // 半径外
    {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 4.0F},
    {-6.0F, -8.0F, 0.0F, 5.0F}, // 边界上，保留
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 3U);
  EXPECT_FLOAT_EQ(output[0].x, 3.0F);
  EXPECT_FLOAT_EQ(output[1].x, 10.0F);
  EXPECT_FLOAT_EQ(output[2].x, -6.0F);
}

TEST(SinglePassFilter, RemovesBodyBoxInteriorWhenNegative)
{
  const auto input = makeCloud({
    {0.0F, 0.0F, 0.0F, 1.0F},    // box 内 → 挖掉
    {0.5F, 0.5F, 0.5F, 2.0F},    // box 外 → 保留
    {2.0F, 0.0F, 0.2F, 3.0F},    // 保留
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 2U);
  EXPECT_FLOAT_EQ(output[0].x, 0.5F);
  EXPECT_FLOAT_EQ(output[1].x, 2.0F);
}

TEST(SinglePassFilter, NegativeFalseKeepsOnlyBoxInterior)
{
  auto params = defaultParams();
  params.negative = false;
  const auto input = makeCloud({
    {0.0F, 0.0F, 0.0F, 1.0F},
    {0.5F, 0.5F, 0.5F, 2.0F},
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), params, output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].x, 0.0F);
}

TEST(SinglePassFilter, JudgesInNavigationFrame)
{
  // navigation 系 = 输入系平移 (5, 0, 0)：输入点 (0,0,0) 在 navigation 系为
  // (5,0,0)，仍在半径内；点 (6,0,0) 变换后为 (11,0,0) 超出半径。
  const auto input = makeCloud({
    {0.0F, 0.0F, 0.0F, 1.0F},
    {6.0F, 0.0F, 0.0F, 2.0F},
  });
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, transform, defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  // 输出坐标仍在输入系
  EXPECT_FLOAT_EQ(output[0].x, 0.0F);
}

TEST(SinglePassFilter, VoxelDownsamplesKeepingFirstPoint)
{
  const auto input = makeCloud({
    {1.000F, 1.000F, 0.0F, 1.0F},
    {1.001F, 1.001F, 0.0F, 2.0F},  // 同一 0.06m 格
    {1.100F, 1.100F, 0.0F, 3.0F},  // 另一格
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 2U);
  EXPECT_FLOAT_EQ(output[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ(output[1].intensity, 3.0F);
}

TEST(SinglePassFilter, PreservesIntensityAndZeroesWhenMissing)
{
  const auto with_intensity = makeCloud({{1.0F, 1.0F, 0.0F, 42.0F}});
  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(
    filterSinglePass(with_intensity, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].intensity, 42.0F);

  const auto without_intensity = makeCloud({{1.0F, 1.0F, 0.0F, 0.0F}}, false);
  ASSERT_TRUE(
    filterSinglePass(without_intensity, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].intensity, 0.0F);
}

TEST(SinglePassFilter, RejectsCloudWithoutXyzFields)
{
  const auto input = makeCloud({{0.0F, 0.0F, 0.0F, 1.0F}}, true, false);
  pcl::PointCloud<pcl::PointXYZI> output;
  EXPECT_FALSE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  EXPECT_TRUE(output.empty());
}

TEST(SinglePassFilter, RejectsBigEndianCloud)
{
  auto input = makeCloud({{1.0F, 1.0F, 0.0F, 1.0F}});
  input.is_bigendian = true;
  pcl::PointCloud<pcl::PointXYZI> output;
  EXPECT_FALSE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
}

}  // namespace cpp_lidar_filter
