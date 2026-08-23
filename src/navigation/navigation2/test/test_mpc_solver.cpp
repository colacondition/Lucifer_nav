#include "mpc/mpc_solver.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

namespace navigation2::mpc
{
namespace
{

MpcParams params(bool path_weights)
{
  MpcParams p;
  p.steps = 12;
  p.dt = 0.1;
  p.max_speed = 3.0;
  p.max_accel = 10.0;
  p.Q = {15.0, 15.0};
  p.R = {0.1, 0.1};
  p.Rd = {0.1, 0.1};
  p.path_frame_weighting = path_weights;
  p.tangential_weight = 6.0;
  p.lateral_weight = 24.0;
  return p;
}

void straightReference(double angle, Eigen::MatrixXd & xref, Eigen::MatrixXd & uref)
{
  const Eigen::Vector2d tangent(std::cos(angle), std::sin(angle));
  for (int i = 0; i < xref.cols(); ++i) {
    xref.col(i) = tangent * (0.15 * i);
    uref.col(i) = tangent * 1.5;
  }
}

TEST(MpcPathFrameWeights, EqualWeightsMatchLegacySolver)
{
  auto legacy_params = params(false);
  auto rotated_params = params(true);
  rotated_params.tangential_weight = 15.0;
  rotated_params.lateral_weight = 15.0;
  MpcSolver legacy;
  MpcSolver rotated;
  legacy.configure(legacy_params);
  rotated.configure(rotated_params);

  Eigen::MatrixXd xref(2, legacy_params.steps);
  Eigen::MatrixXd uref(2, legacy_params.steps);
  straightReference(0.63, xref, uref);
  const Eigen::Vector2d initial(0.2, -0.1);
  const auto a = legacy.solve(xref, uref, initial, false);
  const auto b = rotated.solve(xref, uref, initial, false);

  ASSERT_EQ(a.size(), b.size());
  ASSERT_FALSE(a.empty());
  EXPECT_NEAR(a.front().x(), b.front().x(), 2e-3);
  EXPECT_NEAR(a.front().y(), b.front().y(), 2e-3);
}

TEST(MpcPathFrameWeights, LateralErrorReceivesStrongerCorrection)
{
  const auto p = params(true);
  Eigen::MatrixXd xref(2, p.steps);
  Eigen::MatrixXd uref(2, p.steps);
  straightReference(0.0, xref, uref);

  MpcSolver longitudinal_solver;
  MpcSolver lateral_solver;
  longitudinal_solver.configure(p);
  lateral_solver.configure(p);
  const auto longitudinal = longitudinal_solver.solve(xref, uref, Eigen::Vector2d(0.2, 0.0), false);
  const auto lateral = lateral_solver.solve(xref, uref, Eigen::Vector2d(0.0, 0.2), false);

  ASSERT_FALSE(longitudinal.empty());
  ASSERT_FALSE(lateral.empty());
  const double longitudinal_correction = std::abs(longitudinal.front().x() - uref(0, 0));
  const double lateral_correction = std::abs(lateral.front().y() - uref(1, 0));
  EXPECT_GT(lateral_correction, longitudinal_correction);
}

TEST(MpcPathFrameWeights, RotatingReferenceRotatesCommand)
{
  const auto p = params(true);
  Eigen::MatrixXd xref_a(2, p.steps), uref_a(2, p.steps);
  Eigen::MatrixXd xref_b(2, p.steps), uref_b(2, p.steps);
  straightReference(0.0, xref_a, uref_a);
  const double angle = 0.7;
  straightReference(angle, xref_b, uref_b);
  const Eigen::Rotation2Dd rotation(angle);

  MpcSolver solver_a;
  MpcSolver solver_b;
  solver_a.configure(p);
  solver_b.configure(p);
  const Eigen::Vector2d initial_a(0.1, 0.15);
  const auto out_a = solver_a.solve(xref_a, uref_a, initial_a, false);
  const auto out_b = solver_b.solve(xref_b, uref_b, rotation * initial_a, false);

  ASSERT_FALSE(out_a.empty());
  ASSERT_FALSE(out_b.empty());
  const Eigen::Vector2d expected = rotation * out_a.front();
  EXPECT_NEAR(out_b.front().x(), expected.x(), 3e-3);
  EXPECT_NEAR(out_b.front().y(), expected.y(), 3e-3);
}

}  // namespace
}  // namespace navigation2::mpc
