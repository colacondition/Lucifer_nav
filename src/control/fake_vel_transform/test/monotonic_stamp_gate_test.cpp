// Copyright (c) 2026 colacondition
// Licensed under the MIT License. See LICENSE in the project root.
#include "fake_vel_transform/monotonic_stamp_gate.hpp"

#include <gtest/gtest.h>

TEST(MonotonicStampGateTest, AcceptsOnlyStrictlyIncreasingStamps)
{
  fake_vel_transform::MonotonicStampGate gate;

  EXPECT_TRUE(gate.accept(100));
  EXPECT_FALSE(gate.accept(100));
  EXPECT_FALSE(gate.accept(99));
  EXPECT_TRUE(gate.accept(101));
}
