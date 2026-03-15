#pragma once

#include <cmath>

namespace rm_can_bridge
{

struct FeedbackAimDeg
{
  double yaw_deg;
  double pitch_deg;
};

struct RelativeAimDeg
{
  double yaw_deg;
  double pitch_deg;
};

struct CommandAimDeg
{
  double yaw_deg;
  double pitch_deg;
};

inline double wrapDegrees180(double angle_deg)
{
  double wrapped = std::fmod(angle_deg + 180.0, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  return wrapped - 180.0;
}

inline CommandAimDeg composeAbsoluteAimFromFeedback(
  const FeedbackAimDeg & feedback,
  const RelativeAimDeg & relative)
{
  return CommandAimDeg{
    wrapDegrees180(feedback.yaw_deg + relative.yaw_deg),
    feedback.pitch_deg + relative.pitch_deg,
  };
}

}  // namespace rm_can_bridge
