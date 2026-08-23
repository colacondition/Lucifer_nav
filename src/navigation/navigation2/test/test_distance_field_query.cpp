#include "shared_state.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace navigation2
{
namespace
{

DistanceFieldSnapshot makeQuadraticField()
{
  DistanceFieldSnapshot field;
  field.resolution = 0.2;
  field.origin_x = -1.0;
  field.origin_y = -1.0;
  field.width = 11;
  field.height = 11;
  field.valid = true;
  field.distance.resize(static_cast<std::size_t>(field.width * field.height));
  for (int y = 0; y < field.height; ++y) {
    for (int x = 0; x < field.width; ++x) {
      const double wx = field.origin_x + field.resolution * x;
      const double wy = field.origin_y + field.resolution * y;
      field.distance[static_cast<std::size_t>(y * field.width + x)] =
        static_cast<float>(1.2 + 0.7 * wx + 0.4 * wy + 0.3 * wx * wx - 0.2 * wy * wy + 0.1 * wx * wy);
    }
  }
  return field;
}

TEST(DistanceFieldQuery, QuadraticInterpolationReproducesQuadraticSurface)
{
  const auto field = makeQuadraticField();
  const Eigen::Vector2d pos(0.13, -0.27);
  double distance = 0.0;
  Eigen::Vector2d gradient;

  ASSERT_TRUE(DistanceFieldRegistry::queryQuadratic(field, pos, distance, gradient));

  const double expected_distance =
    1.2 + 0.7 * pos.x() + 0.4 * pos.y() + 0.3 * pos.x() * pos.x() -
    0.2 * pos.y() * pos.y() + 0.1 * pos.x() * pos.y();
  const Eigen::Vector2d expected_gradient(
    0.7 + 0.6 * pos.x() + 0.1 * pos.y(),
    0.4 - 0.4 * pos.y() + 0.1 * pos.x());

  EXPECT_NEAR(distance, expected_distance, 1e-6);
  EXPECT_NEAR(gradient.x(), expected_gradient.x(), 1e-5);
  EXPECT_NEAR(gradient.y(), expected_gradient.y(), 1e-5);
}

TEST(DistanceFieldQuery, QuadraticQueryFallsBackNearMapBoundary)
{
  const auto field = makeQuadraticField();
  const Eigen::Vector2d pos(-0.95, -0.95);
  double bilinear_distance = 0.0;
  double quadratic_distance = 0.0;
  Eigen::Vector2d bilinear_gradient;
  Eigen::Vector2d quadratic_gradient;

  ASSERT_TRUE(DistanceFieldRegistry::query(field, pos, bilinear_distance, bilinear_gradient));
  ASSERT_TRUE(DistanceFieldRegistry::queryQuadratic(
      field, pos, quadratic_distance, quadratic_gradient));

  EXPECT_NEAR(quadratic_distance, bilinear_distance, 1e-12);
  EXPECT_NEAR(quadratic_gradient.x(), bilinear_gradient.x(), 1e-12);
  EXPECT_NEAR(quadratic_gradient.y(), bilinear_gradient.y(), 1e-12);
}

TEST(DistanceFieldQuery, RejectsInvalidResolution)
{
  auto field = makeQuadraticField();
  field.resolution = 0.0;
  double distance = 0.0;
  Eigen::Vector2d gradient;
  EXPECT_FALSE(DistanceFieldRegistry::queryQuadratic(field, Eigen::Vector2d::Zero(), distance, gradient));
}

}  // namespace
}  // namespace navigation2
