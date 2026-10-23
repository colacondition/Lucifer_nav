#include <array>
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

// 组织化云 + 行尾 padding：height 行、每行 width 个 16 字节点。
// padding 字节填 0xAB —— 若实现错误地从 padding 处读坐标，会读出差值明显的垃圾。
sensor_msgs::msg::PointCloud2 makePaddedCloud(
  const std::vector<std::array<float, 4>> & points, uint32_t width, uint32_t row_padding)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = static_cast<uint32_t>(points.size() / width);
  msg.width = width;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 16U;
  msg.row_step = msg.width * msg.point_step + row_padding;
  msg.data.assign(static_cast<std::size_t>(msg.row_step) * msg.height, 0xAB);

  sensor_msgs::msg::PointField f;
  f.count = 1;
  f.datatype = sensor_msgs::msg::PointField::FLOAT32;
  f.name = "x"; f.offset = 0; msg.fields.push_back(f);
  f.name = "y"; f.offset = 4; msg.fields.push_back(f);
  f.name = "z"; f.offset = 8; msg.fields.push_back(f);
  f.name = "intensity"; f.offset = 12; msg.fields.push_back(f);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t row = i / width;
    const std::size_t column = i % width;
    const std::size_t offset =
      row * static_cast<std::size_t>(msg.row_step) + column * msg.point_step;
    std::memcpy(msg.data.data() + offset, points[i].data(), sizeof(float) * 4);
  }
  return msg;
}

// x/y/z 为 FLOAT64 的云（无 intensity）。
sensor_msgs::msg::PointCloud2 makeFloat64Cloud(
  const std::vector<std::array<double, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.point_step = 24U;
  msg.row_step = msg.width * msg.point_step;
  msg.data.resize(msg.row_step);

  sensor_msgs::msg::PointField f;
  f.count = 1;
  f.datatype = sensor_msgs::msg::PointField::FLOAT64;
  f.name = "x"; f.offset = 0; msg.fields.push_back(f);
  f.name = "y"; f.offset = 8; msg.fields.push_back(f);
  f.name = "z"; f.offset = 16; msg.fields.push_back(f);

  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(
      msg.data.data() + i * msg.point_step, points[i].data(), sizeof(double) * 3);
  }
  return msg;
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

// PointCloud2 允许 row_step > width*point_step（行尾对齐 padding）。旧实现要求
// 严格 packed，会把合法的组织化云整帧丢弃。
TEST(SinglePassFilter, AcceptsPaddedRowStepOrganizedCloud)
{
  const std::vector<std::array<float, 4>> points{
    {1.0F, 1.0F, 0.0F, 1.0F},
    {2.0F, 1.0F, 0.0F, 2.0F},
    {3.0F, 1.0F, 0.0F, 3.0F},
    {4.0F, 1.0F, 0.0F, 4.0F},
  };
  const auto input = makePaddedCloud(points, 2U, 8U);
  ASSERT_EQ(input.height, 2U);

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 4U);
  EXPECT_FLOAT_EQ(output[0].x, 1.0F);
  // 第二行必须从 row_step 处开始读；读错会拿第二行第一列当 x=3。
  EXPECT_FLOAT_EQ(output[2].x, 3.0F);
  EXPECT_FLOAT_EQ(output[2].intensity, 3.0F);
}

TEST(SinglePassFilter, AcceptsLastRowWithoutPadding)
{
  const std::vector<std::array<float, 4>> points{
    {1.0F, 1.0F, 0.0F, 1.0F},
    {2.0F, 1.0F, 0.0F, 2.0F},
    {3.0F, 1.0F, 0.0F, 3.0F},
    {4.0F, 1.0F, 0.0F, 4.0F},
  };
  auto input = makePaddedCloud(points, 2U, 8U);
  // 部分实现不补最后一行的 padding：data 长度 = row_step*(h-1) + packed_row_step。
  input.data.resize(
    static_cast<std::size_t>(input.row_step) * (input.height - 1U) +
    static_cast<std::size_t>(input.width) * input.point_step);

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  EXPECT_EQ(output.size(), 4U);
}

TEST(SinglePassFilter, RejectsTruncatedOrganizedData)
{
  const std::vector<std::array<float, 4>> points{
    {1.0F, 1.0F, 0.0F, 1.0F},
    {2.0F, 1.0F, 0.0F, 2.0F},
    {3.0F, 1.0F, 0.0F, 3.0F},
    {4.0F, 1.0F, 0.0F, 4.0F},
  };
  auto input = makePaddedCloud(points, 2U, 8U);
  // 截到连最后一行 packed 数据都放不下：必须拒绝，而不是越界读。
  const std::size_t last_row_without_padding =
    static_cast<std::size_t>(input.row_step) * (input.height - 1U) +
    static_cast<std::size_t>(input.width) * input.point_step;
  input.data.resize(last_row_without_padding - 1U);

  pcl::PointCloud<pcl::PointXYZI> output;
  EXPECT_FALSE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
}

TEST(SinglePassFilter, EmptyCloudIsValidAndProducesEmptyOutput)
{
  sensor_msgs::msg::PointCloud2 input;
  input.height = 1;
  input.width = 0;
  input.is_bigendian = false;
  input.is_dense = true;
  input.point_step = 16U;
  input.row_step = 0U;
  sensor_msgs::msg::PointField f;
  f.count = 1;
  f.datatype = sensor_msgs::msg::PointField::FLOAT32;
  f.name = "x"; f.offset = 0; input.fields.push_back(f);
  f.name = "y"; f.offset = 4; input.fields.push_back(f);
  f.name = "z"; f.offset = 8; input.fields.push_back(f);

  pcl::PointCloud<pcl::PointXYZI> output;
  // 空点云是有效输入（例如上游短暂空帧），不该当成解析错误整帧丢弃。
  EXPECT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  EXPECT_TRUE(output.empty());
}

TEST(SinglePassFilter, AcceptsFloat64XyzFields)
{
  const auto input = makeFloat64Cloud({
    {1.0, 1.0, 0.0},  // box 外 → 保留
    {0.0, 0.0, 0.0},  // box 内 → 挖掉
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output[0].x, 1.0, 1e-6);
  EXPECT_NEAR(output[0].y, 1.0, 1e-6);
}

// 判定必须发生在「变换到 navigation 系之后」：绕 z 旋转 90° 时，输入系里的
// (0,-0.2,0.2) 在 navigation 系是 (0.2,0,0.2)（box 内），(0,-1,0.2) 是
// (1,0,0.2)（box 外）。输出坐标仍留在输入系。
TEST(SinglePassFilter, RotationAppliedBeforeBoxJudgement)
{
  const auto input = makeCloud({
    {0.0F, -0.2F, 0.2F, 1.0F},  // 变换后 box 内 → 挖掉
    {0.0F, -1.0F, 0.2F, 2.0F},  // 变换后 box 外 → 保留
  });
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() =
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, transform, defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].y, -1.0F);   // 输出是输入系原始坐标
  EXPECT_FLOAT_EQ(output[0].z, 0.2F);
}

TEST(SinglePassFilter, BoxBoundaryIsInclusive)
{
  // 用二进制可精确表示的 box 边界（0.5/0.25/0.125/0.625），避免 float→double
  // 舍入把"正好在边界上"变成"差 1ulp 在外"。
  auto params = defaultParams();
  params.body_min_x = -0.5;
  params.body_max_x = 0.5;
  params.body_min_y = -0.25;
  params.body_max_y = 0.25;
  params.body_min_z = -0.25;
  params.body_max_z = 0.5;

  const auto input = makeCloud({
    {0.5F, 0.25F, 0.5F, 1.0F},     // 右后上边界（含）→ 挖掉
    {-0.5F, -0.25F, -0.25F, 2.0F}, // 左前下边界（含）→ 挖掉
    {0.625F, 0.0F, 0.0F, 3.0F},    // x 超 max_x → 保留
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), params, output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].x, 0.625F);
}

TEST(SinglePassFilter, DropsNonFinitePointsAndSanitizesIntensity)
{
  const auto input = makeCloud({
    {std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 1.0F},  // inf → 丢
    {1.0F, 1.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()},  // 保留，intensity→0
    {1.0F, 2.0F, std::numeric_limits<float>::quiet_NaN(), 2.0F},  // NaN z → 丢
  });

  pcl::PointCloud<pcl::PointXYZI> output;
  ASSERT_TRUE(filterSinglePass(input, Eigen::Affine3d::Identity(), defaultParams(), output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output[0].intensity, 0.0F);
}

}  // namespace cpp_lidar_filter
