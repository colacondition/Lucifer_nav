#include <gtest/gtest.h>

#include "decision/decision_context.hpp"

TEST(IntegrityState, EmptyAndNonLostAreOpen)
{
  EXPECT_FALSE(decision::integrityStateIsLost(""));
  EXPECT_FALSE(decision::integrityStateIsLost("OK accepted"));
  EXPECT_FALSE(decision::integrityStateIsLost("OK nis_reject"));
  EXPECT_FALSE(decision::integrityStateIsLost("OK projected"));
  EXPECT_FALSE(decision::integrityStateIsLost("OK weak_obs"));
  EXPECT_TRUE(decision::integrityStateIsLost("LOST"));
  EXPECT_TRUE(decision::integrityStateIsLost("LOST waiting"));
  EXPECT_TRUE(decision::integrityStateIsLost("LOST tracking_lost"));
}

TEST(DecisionContext, NeverReceivedStatusIsNotLost)
{
  decision::DecisionContext ctx;
  EXPECT_FALSE(ctx.hasLocalizationStatus());
  EXPECT_FALSE(ctx.localizationLost());
  EXPECT_TRUE(ctx.localizationStatusPayload().empty());
}

TEST(DecisionContext, LostFreezesAndOkClears)
{
  decision::DecisionContext ctx;
  ctx.setLocalizationStatus("LOST waiting");
  EXPECT_TRUE(ctx.hasLocalizationStatus());
  EXPECT_TRUE(ctx.localizationLost());
  EXPECT_EQ(ctx.localizationStatusPayload(), "LOST waiting");

  ctx.setLocalizationStatus("OK nis_reject");
  EXPECT_FALSE(ctx.localizationLost());

  ctx.setLocalizationStatus("OK accepted");
  EXPECT_FALSE(ctx.localizationLost());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
