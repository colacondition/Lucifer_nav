#include <gtest/gtest.h>
#include <rmw/qos_profiles.h>

#include "decision/decision_state.hpp"

TEST(DecisionState, FormatsEveryLeafState)
{
  EXPECT_EQ(decision::toString(decision::HomeState{decision::HomeSubstate::WaitHome}), "HOME.WAIT_HOME");
  EXPECT_EQ(decision::toString(decision::HomeState{decision::HomeSubstate::WaitHp}), "HOME.WAIT_HP");
  EXPECT_EQ(decision::toString(decision::MoveState{decision::MoveSubstate::GoHome}), "MOVE.GO_HOME");
  EXPECT_EQ(decision::toString(decision::MoveState{decision::MoveSubstate::GoCenter}), "MOVE.GO_CENTER");
  EXPECT_EQ(decision::toString(decision::CenterState{decision::CenterSubstate::WaitCenter}), "CENTER.WAIT_CENTER");
  EXPECT_EQ(decision::toString(decision::CenterState{decision::CenterSubstate::Patrol}), "CENTER.PATROL");
}

TEST(DecisionState, MapsEveryLeafStateToOneTarget)
{
  EXPECT_EQ(decision::targetForState(decision::HomeState{decision::HomeSubstate::WaitHome}), decision::TargetName::WaitHome);
  EXPECT_EQ(decision::targetForState(decision::HomeState{decision::HomeSubstate::WaitHp}), decision::TargetName::WaitHp);
  EXPECT_EQ(decision::targetForState(decision::MoveState{decision::MoveSubstate::GoHome}), decision::TargetName::Home);
  EXPECT_EQ(decision::targetForState(decision::MoveState{decision::MoveSubstate::GoCenter}), decision::TargetName::Center);
  EXPECT_EQ(decision::targetForState(decision::CenterState{decision::CenterSubstate::WaitCenter}), decision::TargetName::WaitCenter);
  EXPECT_EQ(decision::targetForState(decision::CenterState{decision::CenterSubstate::Patrol}), decision::TargetName::Patrol);
}

TEST(DecisionState, BuildsLatchedWebMessage)
{
  const decision::DecisionState state = decision::MoveState{decision::MoveSubstate::GoCenter};
  EXPECT_EQ(decision::decisionStateMessage(state).data, "MOVE.GO_CENTER");
  const auto profile = decision::decisionStateQos().get_rmw_qos_profile();
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}
