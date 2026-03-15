#pragma once

#include <cmath>
#include <stdexcept>

namespace rm_can_bridge
{

struct Vec3
{
  double x;
  double y;
  double z;
};

struct AimAngles
{
  double yaw_rad;
  double pitch_rad;
};

inline AimAngles computeRelativeAim(const Vec3 & target, const Vec3 & muzzle)
{
  const double dx = target.x - muzzle.x;
  const double dy = target.y - muzzle.y;
  const double dz = target.z - muzzle.z;
  const double planar = std::hypot(dx, dy);

  if (planar < 1e-9 && std::abs(dz) < 1e-9) {
    throw std::runtime_error("target and muzzle positions coincide");
  }

  return AimAngles{
    std::atan2(dy, dx),
    std::atan2(dz, planar),
  };
}


}  // namespace rm_can_bridge
