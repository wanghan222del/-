#ifndef ARMOR_DETECTOR__LATEST_FRAME_SLOT_HPP_
#define ARMOR_DETECTOR__LATEST_FRAME_SLOT_HPP_

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <utility>

namespace rm_auto_aim
{

template <typename T>
class LatestFrameSlot
{
public:
  void push(const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    overwriteLocked();
    value_ = value;
    has_value_ = true;
    cv_.notify_one();
  }

  void push(T && value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    overwriteLocked();
    value_ = std::move(value);
    has_value_ = true;
    cv_.notify_one();
  }

  bool try_take(T & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_value_) {
      return false;
    }
    out = std::move(value_);
    has_value_ = false;
    return true;
  }

  bool wait_and_take(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return stopped_ || has_value_; });
    if (!has_value_) {
      return false;
    }
    out = std::move(value_);
    has_value_ = false;
    return true;
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    cv_.notify_all();
  }

  std::size_t overwritten_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return overwritten_count_;
  }

private:
  void overwriteLocked()
  {
    if (has_value_) {
      ++overwritten_count_;
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  T value_{};
  bool has_value_{false};
  bool stopped_{false};
  std::size_t overwritten_count_{0};
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__LATEST_FRAME_SLOT_HPP_
