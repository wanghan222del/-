#pragma once

#include <array>
#include <cstdint>

#include "rm_can_bridge/q10_6_angle.hpp"

namespace rm_can_bridge
{

struct GimbalCommandInput
{
  bool e_stop;
  bool aim_fresh;
  bool fire_enable;
  bool auto_fire;
  bool hold_last_aim;
  bool has_last_aim;
  int16_t yaw_q10_6;
  int16_t pitch_q10_6;
};

inline std::array<uint8_t, 8> buildGimbalCommandPayload(const GimbalCommandInput & input)
{
  std::array<uint8_t, 8> data{};

  const bool should_send_aim =
    !input.e_stop && (input.aim_fresh || (input.hold_last_aim && input.has_last_aim));
  const bool fire =
    !input.e_stop && (input.fire_enable || (input.auto_fire && input.aim_fresh));

  if (should_send_aim) {
    writeInt16BE(data, 0, input.yaw_q10_6);
    writeInt16BE(data, 2, input.pitch_q10_6);
  }

  data[4] = fire ? 1U : 0U;
  uint8_t flags = 0;
  if (input.aim_fresh) {
    flags |= (1U << 0);
  }
  if (fire) {
    flags |= (1U << 1);
  }
  data[5] = flags;

  return data;
}

}  // namespace rm_can_bridge
