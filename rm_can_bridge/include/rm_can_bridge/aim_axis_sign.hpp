#pragma once

namespace rm_can_bridge
{

struct AimAxisSignInputDeg
{
  double yaw_deg;
  double pitch_deg;
};

struct AimAxisSignConfig
{
  bool invert_yaw;
  bool invert_pitch;
};

inline AimAxisSignInputDeg applyAimAxisSign(
  const AimAxisSignInputDeg & input, const AimAxisSignConfig & config)
{
  return AimAxisSignInputDeg{
    config.invert_yaw ? -input.yaw_deg : input.yaw_deg,
    config.invert_pitch ? -input.pitch_deg : input.pitch_deg,
  };
}

}  // namespace rm_can_bridge
