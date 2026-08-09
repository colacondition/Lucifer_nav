#pragma once

#include <cstdint>
#include <string>

namespace decision
{

enum class TargetName
{
  WaitHome,
  WaitHp,
  Home,
  Patrol,
  Center,
  WaitCenter,
  Unknown
};

enum class TargetMode
{
  Disabled,
  ExecutorFollow,
  ExecutorThrough
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct TimedPose
{
  Pose pose;
  double stamp_sec{-1.0e9};
  std::string source;
};

std::string toString(TargetName target);
TargetName targetNameFromString(const std::string & value);

}  // namespace decision
