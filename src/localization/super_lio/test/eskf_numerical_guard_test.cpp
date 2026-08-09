#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lio/ESKF.h"

namespace
{

LI2Sup::IMUData makeImu(double timestamp, double acceleration)
{
  LI2Sup::IMUData imu;
  imu.secs = timestamp;
  imu.acc = BASIC::V3(0.0f, 0.0f, acceleration);
  imu.gyr = BASIC::V3::Zero();
  return imu;
}

LI2Sup::ESKF makeInitializedEskf(double timestamp)
{
  LI2Sup::ESKF eskf;
  eskf.SetInitialConditions(
    LI2Sup::ESKF::Options(),
    BASIC::V3::Zero(),
    BASIC::V3::Zero(),
    1.0f,
    BASIC::V3(0.0f, 0.0f, -9.81f));
  eskf.SetX(LI2Sup::SysState(
    timestamp,
    BASIC::SO3(),
    BASIC::V3::Zero(),
    BASIC::V3::Zero()));
  eskf.init_ = true;
  return eskf;
}

}

TEST(EskfNumericalGuardTest, RejectsNonFiniteImuWithoutPoisoningPrediction)
{
  auto eskf = makeInitializedEskf(10.0);
  LI2Sup::DynamicState imu_state;
  LI2Sup::DynamicState robot_state;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(eskf.Predict(makeImu(10.01, nan), imu_state, robot_state));
  EXPECT_FALSE(eskf.Predict(makeImu(10.02, 9.81), imu_state, robot_state));
  EXPECT_TRUE(eskf.Predict(makeImu(10.03, 9.81), imu_state, robot_state));
  EXPECT_TRUE(imu_state.p.allFinite());
  EXPECT_TRUE(robot_state.p.allFinite());
}

TEST(EskfNumericalGuardTest, RecoversAfterImuTimestampRollback)
{
  auto eskf = makeInitializedEskf(10.0);
  LI2Sup::DynamicState imu_state;
  LI2Sup::DynamicState robot_state;

  EXPECT_FALSE(eskf.Predict(makeImu(10.01, 9.81), imu_state, robot_state));
  EXPECT_TRUE(eskf.Predict(makeImu(10.02, 9.81), imu_state, robot_state));
  EXPECT_FALSE(eskf.Predict(makeImu(1.00, 9.81), imu_state, robot_state));
  EXPECT_TRUE(eskf.Predict(makeImu(1.01, 9.81), imu_state, robot_state));
  EXPECT_TRUE(imu_state.p.allFinite());
}

TEST(EskfNumericalGuardTest, RejectsNonFiniteObservationUpdate)
{
  auto eskf = makeInitializedEskf(10.0);

  const bool updated = eskf.UpdateObserve(
    [](const LI2Sup::ESKF::KFState &, BASIC::M6 &htvh, BASIC::V6 &htvr) {
      htvh.setConstant(std::numeric_limits<float>::quiet_NaN());
      htvr.setZero();
    });

  EXPECT_FALSE(updated);
  EXPECT_TRUE(eskf.GetNavState().p.allFinite());
  EXPECT_TRUE(eskf.GetNavState().R.R_.allFinite());
}
