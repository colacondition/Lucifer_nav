#pragma once
#include <chrono>

namespace navigation2::utils {

// 发布器存在且有人订阅时才返回真。
template<class PublisherT>
inline bool publisher_sub(const PublisherT & publisher) noexcept
{
  return publisher && publisher->get_subscription_count() > 0;
}

}  // namespace navigation2::utils
