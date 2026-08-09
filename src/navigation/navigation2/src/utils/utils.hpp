#pragma once
#include <chrono>

namespace navigation2::utils {

// 发布器存在且有人订阅时才返回真。
template<class PublisherT>
inline bool publisher_sub(const PublisherT & publisher) noexcept
{
  return publisher && publisher->get_subscription_count() > 0;
}

// 限制调用频率。
template<typename Func>
void dt_once(Func && func, std::chrono::duration<double> dt) noexcept
{
  static auto last_call = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (now - last_call >= dt) {
    last_call = now;
    func();
  }
}

}  // namespace navigation2::utils
