#include "distance_transform.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace navigation2
{
namespace
{

TEST(DistanceTransform, ReturnsExactAxisAndDiagonalDistance)
{
  constexpr int width = 5;
  constexpr int height = 5;
  std::vector<std::uint8_t> seeds(width * height, 0);
  seeds[2 * width + 2] = 1;
  DistanceTransformWorkspace workspace;
  std::vector<double> output;

  exactDistanceTransform(seeds, width, height, workspace, output);

  ASSERT_EQ(output.size(), seeds.size());
  EXPECT_NEAR(output[2 * width + 3], 1.0, 1e-12);
  EXPECT_NEAR(output[3 * width + 3], std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(output[0], std::sqrt(8.0), 1e-12);
}

TEST(DistanceTransform, ReusesWorkspaceCapacityAtFixedSize)
{
  constexpr int width = 20;
  constexpr int height = 15;
  std::vector<std::uint8_t> seeds(width * height, 0);
  seeds[10] = 1;
  DistanceTransformWorkspace workspace;
  std::vector<double> output;
  exactDistanceTransform(seeds, width, height, workspace, output);

  const auto squared_capacity = workspace.squared.capacity();
  const auto f_capacity = workspace.f.capacity();
  const auto output_capacity = output.capacity();
  seeds[10] = 0;
  seeds[100] = 1;
  exactDistanceTransform(seeds, width, height, workspace, output);

  EXPECT_EQ(workspace.squared.capacity(), squared_capacity);
  EXPECT_EQ(workspace.f.capacity(), f_capacity);
  EXPECT_EQ(output.capacity(), output_capacity);
}

TEST(DistanceTransform, InvalidGeometryReturnsEmptyOutput)
{
  DistanceTransformWorkspace workspace;
  std::vector<double> output{1.0};
  exactDistanceTransform({}, 0, 0, workspace, output);
  EXPECT_TRUE(output.empty());
}

}  // namespace
}  // namespace navigation2
