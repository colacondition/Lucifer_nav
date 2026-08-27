#include "grid_utils.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace
{

// 造一张随机占据栅格：稀疏障碍 + 少量 unknown(-1)。
nav_msgs::msg::OccupancyGrid makeRandomGrid(
  int width, int height, double resolution, float occupancy_ratio, std::uint32_t seed)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -2.0;
  grid.data.resize(static_cast<std::size_t>(width) * height, 0);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (auto & cell : grid.data) {
    const double r = u(rng);
    if (r < occupancy_ratio) {
      cell = 100;
    } else if (r < occupancy_ratio + 0.03) {
      cell = -1;  // unknown，两版实现都必须原样保留
    }
  }
  return grid;
}

// 参考卷积实现，逐字节断言 EDT 版本等价。
TEST(EdtInflation, BinaryMatchesConvolutionByteForByte)
{
  for (const auto seed : {11u, 42u, 2024u}) {
    auto base = makeRandomGrid(41, 33, 0.05, 0.08, seed);
    auto conv = base;
    auto edt = base;

    navigation2::inflateOccupancyGrid(conv, 0.30, 50);
    navigation2::inflateOccupancyGridEDT(edt, 0.30, 50);

    ASSERT_EQ(conv.data.size(), edt.data.size());
    for (std::size_t i = 0; i < conv.data.size(); ++i) {
      ASSERT_EQ(conv.data[i], edt.data[i])
          << "seed=" << seed << " idx=" << i
          << " conv=" << static_cast<int>(conv.data[i])
          << " edt=" << static_cast<int>(edt.data[i]);
    }
  }
}

TEST(EdtInflation, CostGradientMatchesConvolutionByteForByte)
{
  const std::vector<float> empty_limit;
  // 有隧道的情形再跑一遍带逐格上限的路径（含 size 对不上时整体忽略的分支）。
  for (const bool with_limit : {false, true}) {
    for (const auto seed : {7u, 99u}) {
      auto base = makeRandomGrid(38, 52, 0.05, 0.10, seed);
      std::vector<float> limits;
      if (with_limit) {
        limits.assign(base.data.size(), 0.18F);
        // 一半格子放宽到全局半径，制造两种条件的混合。
        for (std::size_t i = 0; i < limits.size(); i += 3) {
          limits[i] = 0.6F;
        }
      }

      auto conv = base;
      auto edt = base;
      navigation2::applyInflationCostGradient(conv, 0.35, 50, 15.0, limits);
      navigation2::applyInflationCostGradientEDT(edt, 0.35, 50, 15.0, limits);

      ASSERT_EQ(conv.data.size(), edt.data.size());
      for (std::size_t i = 0; i < conv.data.size(); ++i) {
        ASSERT_EQ(conv.data[i], edt.data[i])
            << "with_limit=" << with_limit << " seed=" << seed << " idx=" << i;
      }
    }
  }
}

TEST(EdtInflation, DegenerateInputsMatchConvolution)
{
  // 半径 0 / 空图 / 分辨率非法：两者都必须原样返回。
  for (double radius : {0.0, -0.5}) {
    auto base = makeRandomGrid(16, 16, 0.05, 0.1, 1u);
    auto conv = base;
    auto edt = base;
    navigation2::inflateOccupancyGrid(conv, radius);
    navigation2::inflateOccupancyGridEDT(edt, radius);
    EXPECT_EQ(conv.data, edt.data);
  }

  nav_msgs::msg::OccupancyGrid empty;
  empty.info.width = 0;
  empty.info.height = 0;
  empty.info.resolution = 0.05F;
  navigation2::inflateOccupancyGrid(empty, 0.3);
  navigation2::inflateOccupancyGridEDT(empty, 0.3);
  EXPECT_TRUE(empty.data.empty());
}

}  // namespace
