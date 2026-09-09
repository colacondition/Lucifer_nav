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

// enforce_speed_norm = true 时，QP 的解在任何方向上的模长都不得超过 max_speed。
// 参考「斜 45°、参考速度远超上限」，旧 box 语义下解会贴到 (vmax, vmax)，
// 模长 sqrt(2)*vmax；八边形语义下必须被压回 vmax 以内。
TEST(MpcSpeedNorm, DiagonalSpeedCappedAtMaxSpeed)
{
  auto box_params = params(false);
  auto norm_params = params(false);
  // 显式打开历史语义：默认已是八边形，这里要对比的是被修掉的那个行为。
  box_params.enforce_speed_norm = false;
  norm_params.enforce_speed_norm = true;
  MpcSolver box_solver;
  MpcSolver norm_solver;
  box_solver.configure(box_params);
  norm_solver.configure(norm_params);

  Eigen::MatrixXd xref(2, box_params.steps);
  Eigen::MatrixXd uref(2, box_params.steps);
  // 45° 方向、4.2 m/s 参考速度（模长已超 max_speed），box 语义下各分量饱和到
  // vmax，模长 sqrt(2)*vmax。
  for (int i = 0; i < xref.cols(); ++i) {
    const double d = 4.2 * 0.1 * i;
    xref.col(i) = Eigen::Vector2d(d * std::sqrt(0.5), d * std::sqrt(0.5));
    uref.col(i) = Eigen::Vector2d(4.2 * std::sqrt(0.5), 4.2 * std::sqrt(0.5));
  }
  const Eigen::Vector2d initial(0.0, 0.0);

  const auto box_out = box_solver.solve(xref, uref, initial, false);
  const auto norm_out = norm_solver.solve(xref, uref, initial, false);
  ASSERT_FALSE(box_out.empty());
  ASSERT_FALSE(norm_out.empty());

  const double limit = box_params.max_speed;
  // 历史语义确实允许对角超速（这是要修的行为，先钉住它存在）。
  EXPECT_GT(box_out.front().norm(), limit * 1.05);
  // 八边形语义把模长压回声明值。
  EXPECT_LE(norm_out.front().norm(), limit * (1.0 + 1e-6));
  // 内接八边形保留 cos(22.5°) ≈ 0.924 的轴向能力，不该被压到过保守。
  EXPECT_GE(norm_out.front().norm(), limit * 0.5);
}

// 轴向参考下八边形不应把速度压得过低：仍能逼近 max_speed 的 92% 以上。
TEST(MpcSpeedNorm, AxialSpeedRetained)
{
  auto norm_params = params(false);
  norm_params.enforce_speed_norm = true;
  norm_params.max_accel = 50.0;
  MpcSolver solver;
  solver.configure(norm_params);

  Eigen::MatrixXd xref(2, norm_params.steps);
  Eigen::MatrixXd uref(2, norm_params.steps);
  for (int i = 0; i < xref.cols(); ++i) {
    xref.col(i) = Eigen::Vector2d(2.9 * 0.1 * i, 0.0);
    uref.col(i) = Eigen::Vector2d(2.9, 0.0);
  }
  const auto out = solver.solve(xref, uref, Eigen::Vector2d::Zero(), false);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.front().x(), norm_params.max_speed * 0.9238795, 0.05);
  EXPECT_NEAR(out.front().y(), 0.0, 1e-6);
}

}  // namespace
}  // namespace navigation2::mpc
