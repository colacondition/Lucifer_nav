#include <cmath>
#include <cstring>
#include <limits>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cpp_lidar_filter/radial_filter.hpp"

namespace cpp_lidar_filter
{

TEST(RadialFilter, KeepsOnlyFinitePointsInsideOrOnBoundary)
{
  pcl::PointCloud<pcl::PointXYZI> points;
  points.push_back(pcl::PointXYZI{3.0F, 4.0F, 0.0F, 1.0F});
  points.push_back(pcl::PointXYZI{10.0F, 0.0F, 0.0F, 2.0F});
  points.push_back(pcl::PointXYZI{8.0F, 8.0F, 0.0F, 3.0F});
  points.push_back(
    pcl::PointXYZI{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 4.0F});
  points.push_back(pcl::PointXYZI{-6.0F, -8.0F, 0.0F, 5.0F});

  pcl::PCLPointCloud2 cloud;
  pcl::toPCLPointCloud2(points, cloud);

  pcl::Indices indices;
  ASSERT_TRUE(
    collectHorizontalRangeIndices(
      cloud, 10.0, Eigen::Affine3d::Identity(), indices));
  ASSERT_EQ(indices.size(), 3U);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 1);
  EXPECT_EQ(indices[2], 4);
}

TEST(RadialFilter, RejectsCloudWithoutXYFields)
{
  pcl::PCLPointCloud2 cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.point_step = sizeof(float);
  cloud.row_step = cloud.point_step;
  cloud.data.resize(cloud.row_step);

  pcl::Indices indices{7};
  EXPECT_FALSE(
    collectHorizontalRangeIndices(
      cloud, 10.0, Eigen::Affine3d::Identity(), indices));
  EXPECT_TRUE(indices.empty());
}

TEST(RadialFilter, RejectsRowsWithPadding)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.width = 2;
  points.height = 2;
  points.resize(4);

  pcl::PCLPointCloud2 cloud;
  pcl::toPCLPointCloud2(points, cloud);
  cloud.row_step += cloud.point_step;
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);

  pcl::Indices indices{7};
  EXPECT_FALSE(
    collectHorizontalRangeIndices(
      cloud, 10.0, Eigen::Affine3d::Identity(), indices));
  EXPECT_TRUE(indices.empty());
}

TEST(RadialFilter, SupportsFloat64Coordinates)
{
  pcl::PCLPointCloud2 cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.point_step = 3 * sizeof(double);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = pcl::PCLPointField::FLOAT64;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = sizeof(double);
  cloud.fields[1].datatype = pcl::PCLPointField::FLOAT64;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 2 * sizeof(double);
  cloud.fields[2].datatype = pcl::PCLPointField::FLOAT64;
  cloud.fields[2].count = 1;
  cloud.data.resize(cloud.row_step);

  const double coordinates[][3] = {{6.0, 8.0, 1.0}, {9.0, 9.0, 2.0}, {-10.0, 0.0, 3.0}};
  for (std::size_t index = 0; index < cloud.width; ++index) {
    std::memcpy(
      cloud.data.data() + index * cloud.point_step,
      &coordinates[index][0], sizeof(double));
    std::memcpy(
      cloud.data.data() + index * cloud.point_step + sizeof(double),
      &coordinates[index][1], sizeof(double));
    std::memcpy(
      cloud.data.data() + index * cloud.point_step + 2 * sizeof(double),
      &coordinates[index][2], sizeof(double));
  }

  pcl::Indices indices;
  ASSERT_TRUE(
    collectHorizontalRangeIndices(
      cloud, 10.0, Eigen::Affine3d::Identity(), indices));
  ASSERT_EQ(indices.size(), 2U);
  EXPECT_EQ(indices[0], 0);
  EXPECT_EQ(indices[1], 2);
}

TEST(RadialFilter, EvaluatesRangeAfterTransformingIntoNavigationFrame)
{
  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(pcl::PointXYZ{0.0F, 8.0F, -8.0F});

  pcl::PCLPointCloud2 cloud;
  pcl::toPCLPointCloud2(points, cloud);

  const double pi = std::acos(-1.0);
  const Eigen::Affine3d target_from_cloud(
    Eigen::AngleAxisd(pi / 4.0, Eigen::Vector3d::UnitX()));

  pcl::Indices indices;
  ASSERT_TRUE(
    collectHorizontalRangeIndices(
      cloud, 10.0, target_from_cloud, indices));
  EXPECT_TRUE(indices.empty());
}

}  // namespace cpp_lidar_filter
