#ifndef FAKE_VEL_TRANSFORM__MONOTONIC_STAMP_GATE_HPP_
#define FAKE_VEL_TRANSFORM__MONOTONIC_STAMP_GATE_HPP_

#include <cstdint>
#include <optional>

namespace fake_vel_transform
{

class MonotonicStampGate
{
public:
  bool accept(std::int64_t stamp_nanoseconds)
  {
    if (last_stamp_nanoseconds_ && stamp_nanoseconds <= *last_stamp_nanoseconds_) {
      return false;
    }
    last_stamp_nanoseconds_ = stamp_nanoseconds;
    return true;
  }

private:
  std::optional<std::int64_t> last_stamp_nanoseconds_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__MONOTONIC_STAMP_GATE_HPP_
