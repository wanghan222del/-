#ifndef AUTO_AIM_INTERFACES__LATENCY_WINDOW_HPP_
#define AUTO_AIM_INTERFACES__LATENCY_WINDOW_HPP_

#include <chrono>
#include <cstddef>

namespace auto_aim_interfaces
{

class LatencyWindow
{
public:
  using Clock = std::chrono::steady_clock;

  struct Snapshot
  {
    std::size_t sample_count{0};
    double avg_ms{0.0};
    double max_ms{0.0};
    double last_ms{0.0};
  };

  explicit LatencyWindow(std::chrono::milliseconds period)
  : period_(period), window_start_(Clock::now())
  {
  }

  void observe(double latency_ms)
  {
    ++sample_count_;
    total_ms_ += latency_ms;
    last_ms_ = latency_ms;
    if (sample_count_ == 1 || latency_ms > max_ms_) {
      max_ms_ = latency_ms;
    }
  }

  bool takeSnapshotIfDue(const Clock::time_point & now, Snapshot & snapshot)
  {
    if (now - window_start_ < period_) {
      return false;
    }

    if (sample_count_ == 0) {
      window_start_ = now;
      return false;
    }

    snapshot.sample_count = sample_count_;
    snapshot.avg_ms = total_ms_ / static_cast<double>(sample_count_);
    snapshot.max_ms = max_ms_;
    snapshot.last_ms = last_ms_;

    reset(now);
    return true;
  }

private:
  void reset(const Clock::time_point & now)
  {
    sample_count_ = 0;
    total_ms_ = 0.0;
    max_ms_ = 0.0;
    last_ms_ = 0.0;
    window_start_ = now;
  }

  Clock::duration period_;
  Clock::time_point window_start_;
  std::size_t sample_count_{0};
  double total_ms_{0.0};
  double max_ms_{0.0};
  double last_ms_{0.0};
};

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__LATENCY_WINDOW_HPP_
