#include "semantic_map.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace navigation2
{
namespace
{

constexpr double kRes = 0.05;

std::uint8_t encodeAngle(double radians)
{
  while (radians < 0.0) {
    radians += 2.0 * M_PI;
  }
  while (radians >= 2.0 * M_PI) {
    radians -= 2.0 * M_PI;
  }
  return static_cast<std::uint8_t>(std::lround(radians / (2.0 * M_PI) * 255.0));
}

SemanticMapData makeData(int width, int height)
{
  SemanticMapData data;
  data.geometry.width = width;
  data.geometry.height = height;
  data.geometry.resolution = kRes;
  data.geometry.origin = Eigen::Vector2d(-1.0, 2.0);
  const std::size_t cells = data.geometry.cellCount();
  data.terrain.assign(cells, static_cast<std::uint8_t>(TerrainType::FLAT));
  data.direction.assign(cells, 0);
  return data;
}

TunnelSpec makeSpec()
{
  TunnelSpec spec;
  spec.clear_height = 0.35;
  spec.clear_width = 0.8;
  spec.run_up = 0.6;
  spec.velocity_min = 0.2;
  spec.velocity_max = 0.6;
  return spec;
}

InflationParams makeParams()
{
  InflationParams params;
  params.resolution = kRes;
  params.full_cost_radius_m = 0.10;
  params.cutoff_radius_m = 0.30;
  params.decay_rate_per_m = 24.0;
  params.non_body_magnitude_cap = kMaxInflatedMagnitude;
  return params;
}

TEST(SemanticMapGeometry, MapsPointsToCellsAgainstNonZeroOrigin)
{
  const auto data = makeData(10, 8);
  const auto & geometry = data.geometry;

  // origin 是栅格 (0,0) 的角点，覆盖 [origin, origin + size * res)。
  EXPECT_TRUE(geometry.containsPoint(Eigen::Vector2d(-1.0, 2.0)));
  EXPECT_FALSE(geometry.containsPoint(Eigen::Vector2d(-1.001, 2.0)));
  EXPECT_FALSE(geometry.containsPoint(Eigen::Vector2d(-1.0 + 10 * kRes, 2.0)));

  const auto cell = geometry.containingCell(Eigen::Vector2d(-1.0 + 2.5 * kRes, 2.0 + 3.5 * kRes));
  ASSERT_TRUE(cell.has_value());
  EXPECT_EQ(cell->x(), 2);
  EXPECT_EQ(cell->y(), 3);
  EXPECT_FALSE(geometry.containingCell(Eigen::Vector2d(0.0, 0.0)).has_value());

  const Eigen::Vector2d center = geometry.cellCenter(2, 3);
  EXPECT_NEAR(center.x(), -1.0 + 2.5 * kRes, 1e-12);
  EXPECT_NEAR(center.y(), 2.0 + 3.5 * kRes, 1e-12);
}

TEST(SemanticMapInflation, CostDecaysByEuclideanDistanceNotGridSteps)
{
  auto data = makeData(41, 41);
  data.terrain[data.geometry.index(20, 20)] = static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const auto params = makeParams();
  const SemanticMap map = SemanticMap::inflate(data, params);
  ASSERT_TRUE(map.valid());

  EXPECT_EQ(map.costAtCell(20, 20), 255);
  // 满代价半径 0.10 m = 2 格，含 2 格。
  EXPECT_EQ(map.costAtCell(22, 20), 255);
  // 3 格 = 0.15 m，超出满代价半径开始衰减。
  const double expected3 = 255.0 * std::exp(-24.0 * (3 * kRes - 0.10));
  EXPECT_NEAR(map.costAtCell(23, 20), std::lround(expected3), 1.0);
  // 截断半径 0.30 m = 6 格之外为 0。
  EXPECT_EQ(map.costAtCell(27, 20), 0);

  // 对角线用真欧氏距离：(3,3) 是 3*sqrt(2)=4.24 格，不是 Dijkstra 八邻域的 3 格。
  const double diagonal_m = std::hypot(3.0, 3.0) * kRes;
  const double expected_diagonal = 255.0 * std::exp(-24.0 * (diagonal_m - 0.10));
  EXPECT_NEAR(map.costAtCell(23, 23), std::lround(expected_diagonal), 1.0);
  EXPECT_LT(map.costAtCell(23, 23), map.costAtCell(23, 20));
}

TEST(SemanticMapInflation, UnknownCellsAreNotObstacleSeeds)
{
  auto data = makeData(21, 21);
  data.terrain[data.geometry.index(10, 10)] = static_cast<std::uint8_t>(TerrainType::UNKNOWN);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());

  // UNKNOWN 只是没扫到，不是有东西。膨胀不把它当障碍，惩罚交给 A* 的 unknown_cost。
  EXPECT_EQ(map.costAtCell(10, 10), 0);
  EXPECT_EQ(map.terrainAtCell(10, 10), static_cast<std::uint8_t>(TerrainType::UNKNOWN));
}

TEST(SemanticMapInflation, TunnelBodyKeepsLabelledAxisAtFullMagnitude)
{
  auto data = makeData(31, 31);
  data.tunnels.push_back(makeSpec());
  // 沿 x 轴一条隧道，轴线指向 +x。
  for (int x = 10; x <= 20; ++x) {
    const std::size_t index = data.geometry.index(x, 15);
    data.terrain[index] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
    data.direction[index] = encodeAngle(0.0);
  }

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  ASSERT_TRUE(map.valid());

  EXPECT_TRUE(map.isTunnelBodyCell(15, 15));
  EXPECT_GT(map.magnitudeAtCell(15, 15), kTerrainBodyMagnitudeThreshold);
  EXPECT_NEAR(map.directionAtCell(15, 15).x(), 1.0, 1e-9);
  EXPECT_NEAR(map.directionAtCell(15, 15).y(), 0.0, 1e-9);

  const TunnelSpec * spec = map.tunnelSpecAtCell(15, 15);
  ASSERT_NE(spec, nullptr);
  EXPECT_DOUBLE_EQ(spec->clear_height, 0.35);
  EXPECT_EQ(map.tunnelSpecAtCell(0, 0), nullptr);
}

TEST(SemanticMapInflation, InflationRingStaysBelowBodyThreshold)
{
  auto data = makeData(31, 31);
  data.tunnels.push_back(makeSpec());
  for (int x = 10; x <= 20; ++x) {
    const std::size_t index = data.geometry.index(x, 15);
    data.terrain[index] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
    data.direction[index] = encodeAngle(0.0);
  }

  const SemanticMap map = SemanticMap::inflate(data, makeParams());

  // 紧邻本体的格子衰减系数是 1.0，但必须被 cap 压到 0.9 —— 「本体」判定不能靠
  // 距离，只能靠标签，否则膨胀圈会被当成车已经在隧道里。
  EXPECT_NEAR(map.magnitudeAtCell(15, 16), kMaxInflatedMagnitude, 1e-9);
  EXPECT_LT(map.magnitudeAtCell(15, 16), kTerrainBodyMagnitudeThreshold);
  EXPECT_FALSE(map.isTerrainBodyCell(15, 16));
  EXPECT_FALSE(map.isTunnelBodyCell(15, 16));
  // 膨胀圈仍然携带轴线方向。
  EXPECT_NEAR(std::abs(map.directionAtCell(15, 16).normalized().x()), 1.0, 1e-9);

  // 截断半径外没有方向。
  EXPECT_NEAR(map.magnitudeAtCell(15, 22), 0.0, 1e-12);
}

TEST(SemanticMapInflation, AveragesAxesAsUndirectedNotAsVectors)
{
  // 这是隧道相对台阶的根本差异。两格轴线标成 0° 和 180°，作为向量相加会抵消成
  // 零向量，膨胀圈就丢了方向；按无向轴（倍角）平均则得到同一条 x 轴。
  auto data = makeData(21, 21);
  data.tunnels.push_back(makeSpec());

  const std::size_t left = data.geometry.index(9, 10);
  const std::size_t right = data.geometry.index(11, 10);
  data.terrain[left] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
  data.direction[left] = encodeAngle(0.0);
  data.terrain[right] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
  data.direction[right] = encodeAngle(M_PI);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());

  // 两个源正中间的格子。作为向量平均这里会是零向量。
  const Eigen::Vector2d axis = map.directionAtCell(10, 10);
  ASSERT_GT(axis.norm(), 0.5);
  // 容差来自角度通道的 uint8 量化：encodeAngle(π) 落在 128/255*2π = 3.1538 rad，
  // 与 0° 那条轴差 0.012 rad，倍角平均后偏一半。这是格式固有精度，不是误差。
  const double kQuantizationTolerance = M_PI / 255.0;
  EXPECT_NEAR(std::abs(axis.normalized().x()), 1.0, kQuantizationTolerance);
  EXPECT_NEAR(axis.normalized().y(), 0.0, kQuantizationTolerance);
}

TEST(SemanticMapInflation, ObstacleCellsCarryNoDirection)
{
  auto data = makeData(21, 21);
  data.tunnels.push_back(makeSpec());
  const std::size_t tunnel = data.geometry.index(10, 10);
  data.terrain[tunnel] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
  data.direction[tunnel] = encodeAngle(0.0);
  data.terrain[data.geometry.index(11, 10)] =
    static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());

  // 车体不可能在障碍格上，给它方向只会污染双线性采样模板。
  EXPECT_NEAR(map.magnitudeAtCell(11, 10), 0.0, 1e-12);
  EXPECT_EQ(map.costAtCell(11, 10), 255);
}

TEST(SemanticMapSampling, CostIsContinuousAcrossCellBoundaries)
{
  auto data = makeData(21, 21);
  data.terrain[data.geometry.index(10, 10)] = static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  const auto & geometry = map.geometry();

  // 跨格边界扫一条线，相邻采样点的差必须是 O(step)，不能有台阶。
  const double step = kRes / 16.0;
  double previous = map.sampleCost(geometry.cellCenter(6, 10)).value;
  for (int i = 1; i <= 16 * 6; ++i) {
    const Eigen::Vector2d point = geometry.cellCenter(6, 10) + Eigen::Vector2d(i * step, 0.0);
    const double current = map.sampleCost(point).value;
    EXPECT_LT(std::abs(current - previous), 40.0) << "discontinuity at i=" << i;
    previous = current;
  }

  // 格心处采样值就等于该格的原始值。
  EXPECT_NEAR(
    map.sampleCost(geometry.cellCenter(10, 10)).value,
    static_cast<double>(map.costAtCell(10, 10)), 1e-9);
  EXPECT_NEAR(
    map.sampleCost(geometry.cellCenter(4, 10)).value,
    static_cast<double>(map.costAtCell(4, 10)), 1e-9);
}

TEST(SemanticMapSampling, CostGradientMatchesFiniteDifference)
{
  auto data = makeData(21, 21);
  data.terrain[data.geometry.index(10, 10)] = static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  // 落在格子内部（不在格心、不在边界），此处双线性是光滑的。
  const Eigen::Vector2d point = map.geometry().cellCenter(7, 10) + Eigen::Vector2d(0.01, 0.007);

  const auto sample = map.sampleCost(point);
  const double eps = 1e-6;
  for (int axis = 0; axis < 2; ++axis) {
    Eigen::Vector2d delta = Eigen::Vector2d::Zero();
    delta(axis) = eps;
    const double numeric =
      (map.sampleCost(point + delta).value - map.sampleCost(point - delta).value) / (2.0 * eps);
    EXPECT_NEAR(sample.gradient(axis), numeric, 1e-3) << "axis " << axis;
  }
}

TEST(SemanticMapSampling, DirectionJacobianMatchesFiniteDifference)
{
  auto data = makeData(21, 21);
  data.tunnels.push_back(makeSpec());
  for (int x = 8; x <= 12; ++x) {
    const std::size_t index = data.geometry.index(x, 10);
    data.terrain[index] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
    data.direction[index] = encodeAngle(0.3);
  }

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  const Eigen::Vector2d point = map.geometry().cellCenter(10, 11) + Eigen::Vector2d(0.013, 0.009);

  const auto sample = map.sampleDirection(point);
  const double eps = 1e-6;
  for (int axis = 0; axis < 2; ++axis) {
    Eigen::Vector2d delta = Eigen::Vector2d::Zero();
    delta(axis) = eps;
    const Eigen::Vector2d numeric =
      (map.sampleDirection(point + delta).value - map.sampleDirection(point - delta).value) /
      (2.0 * eps);
    EXPECT_NEAR(sample.jacobian(0, axis), numeric.x(), 1e-3) << "d vx / d axis " << axis;
    EXPECT_NEAR(sample.jacobian(1, axis), numeric.y(), 1e-3) << "d vy / d axis " << axis;
  }
}

TEST(SemanticMapSampling, LabelWeightsSumToOneAndAreContinuous)
{
  auto data = makeData(21, 21);
  data.tunnels.push_back(makeSpec());
  for (int x = 8; x <= 12; ++x) {
    const std::size_t index = data.geometry.index(x, 10);
    data.terrain[index] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
    data.direction[index] = encodeAngle(0.0);
  }
  data.terrain[data.geometry.index(8, 11)] = static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  const auto & geometry = map.geometry();

  const double step = kRes / 8.0;
  const std::size_t tunnel_label = static_cast<std::size_t>(TerrainType::TUNNEL);
  double previous = map.sampleLabelWeights(geometry.cellCenter(6, 10)).weights[tunnel_label];
  for (int i = 0; i <= 8 * 8; ++i) {
    const Eigen::Vector2d point = geometry.cellCenter(6, 10) + Eigen::Vector2d(i * step, 0.0);
    const auto weights = map.sampleLabelWeights(point);

    double total = 0.0;
    for (double weight : weights.weights) {
      total += weight;
    }
    EXPECT_NEAR(total, 1.0, 1e-9) << "weights must partition unity at i=" << i;

    const double current = weights.weights[tunnel_label];
    EXPECT_LT(std::abs(current - previous), 0.2) << "discontinuity at i=" << i;
    previous = current;
  }

  // 格心处权重全部落在该格的标签上。
  const auto center = map.sampleLabelWeights(geometry.cellCenter(10, 10));
  EXPECT_NEAR(center.weights[tunnel_label], 1.0, 1e-9);
}

TEST(SemanticMapSampling, ReplicatesEdgeValuesAndZeroesOutsideGradient)
{
  auto data = makeData(11, 11);
  data.terrain[data.geometry.index(0, 5)] = static_cast<std::uint8_t>(TerrainType::OBSTACLE);

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  const auto & geometry = map.geometry();

  // 越界点复制边界格的值，并把越界轴的梯度归零 —— 否则优化器会在地图外看到一个
  // 凭空出现的斜坡。
  const Eigen::Vector2d outside = geometry.cellCenter(0, 5) - Eigen::Vector2d(0.5, 0.0);
  const auto sample = map.sampleCost(outside);
  EXPECT_NEAR(sample.value, static_cast<double>(map.costAtCell(0, 5)), 1e-9);
  EXPECT_NEAR(sample.gradient.x(), 0.0, 1e-12);
}

TEST(SemanticMapAlignment, TreatsOppositeHeadingsAsEquallyAligned)
{
  auto data = makeData(21, 21);
  data.tunnels.push_back(makeSpec());
  for (int x = 8; x <= 12; ++x) {
    const std::size_t index = data.geometry.index(x, 10);
    data.terrain[index] = static_cast<std::uint8_t>(TerrainType::TUNNEL);
    data.direction[index] = encodeAngle(0.0);
  }

  const SemanticMap map = SemanticMap::inflate(data, makeParams());
  const Eigen::Vector2d point = map.geometry().cellCenter(10, 10);

  // 双向通道：正着进和倒着进都是完全对齐。
  EXPECT_NEAR(map.axisAlignment(point, Eigen::Vector2d(1.0, 0.0)), 1.0, 1e-9);
  EXPECT_NEAR(map.axisAlignment(point, Eigen::Vector2d(-1.0, 0.0)), 1.0, 1e-9);
  // 垂直穿越完全不对齐。
  EXPECT_NEAR(map.axisAlignment(point, Eigen::Vector2d(0.0, 1.0)), 0.0, 1e-9);
  EXPECT_NEAR(map.axisAlignment(point, Eigen::Vector2d(0.0, -1.0)), 0.0, 1e-9);
  // 45° 斜切。
  EXPECT_NEAR(map.axisAlignment(point, Eigen::Vector2d(1.0, 1.0)), std::sqrt(0.5), 1e-9);

  // 没有方向场的地方不施加约束。
  const Eigen::Vector2d far_away = map.geometry().cellCenter(0, 0);
  EXPECT_NEAR(map.axisAlignment(far_away, Eigen::Vector2d(0.0, 1.0)), 1.0, 1e-9);
}

TEST(SemanticMapInflation, RejectsInconsistentInput)
{
  const auto params = makeParams();

  // 尺寸不匹配。
  auto short_channel = makeData(10, 10);
  short_channel.terrain.pop_back();
  EXPECT_THROW(SemanticMap::inflate(short_channel, params), std::invalid_argument);

  // 分辨率与地图不一致：膨胀半径是物理距离，用错分辨率会静默地把半径缩放掉。
  auto mismatched = makeData(10, 10);
  InflationParams wrong_res = params;
  wrong_res.resolution = 0.10;
  EXPECT_THROW(SemanticMap::inflate(mismatched, wrong_res), std::invalid_argument);

  // cap 超过 0.9 会和「本体」判定撞车。
  InflationParams bad_cap = params;
  bad_cap.non_body_magnitude_cap = 0.99;
  EXPECT_THROW(SemanticMap::inflate(mismatched, bad_cap), std::invalid_argument);

  InflationParams bad_radius = params;
  bad_radius.cutoff_radius_m = 0.05;
  EXPECT_THROW(SemanticMap::inflate(mismatched, bad_radius), std::invalid_argument);
}

}  // namespace
}  // namespace navigation2
