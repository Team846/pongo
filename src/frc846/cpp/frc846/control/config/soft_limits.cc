#include "frc846/control/config/soft_limits.h"

namespace frc846::control::config {

SoftLimits::SoftLimits(bool using_limits, units::radian_t forward_limit,
    units::radian_t reverse_limit, units::radian_t forward_reduce,
    units::radian_t reverse_reduce, double reduce_max_dc)
    : using_limits_(using_limits),
      forward_limit_(forward_limit),
      reverse_limit_(reverse_limit),
      forward_reduce_(forward_reduce),
      reverse_reduce_(reverse_reduce),
      reduce_max_dc_(reduce_max_dc) {
  assert(forward_limit >= reverse_limit &&
         "Forward limit must be greater than reverse limit");
  assert(forward_reduce_ <= forward_limit_ &&
         "Forward reduce must be less than forward limit");
  assert(reverse_reduce_ >= reverse_limit_ &&
         "Reverse reduce must be greater than reverse limit");
  assert(forward_reduce_ >= reverse_reduce_ &&
         "Forward reduce must be greater than reverse reduce");
  assert(reduce_max_dc_ > 0 && "Reduce max DC must be greater than 0");
}

units::radian_t SoftLimits::LimitPosition(units::radian_t position) {
  if (using_limits_ && position < reverse_limit_) {
    return reverse_limit_;
  } else if (using_limits_ && position > forward_limit_) {
    return forward_limit_;
  }
  return position;
}

units::radians_per_second_t SoftLimits::LimitVelocity(
    units::radians_per_second_t velocity, units::radian_t position) {
  if (!using_limits_) return velocity;

  if (velocity < 0_rad_per_s && position <= reverse_limit_) {
    return 0_rad_per_s;
  } else if (velocity > 0_rad_per_s && position >= forward_limit_) {
    return 0_rad_per_s;
  }

  return velocity;
}

double SoftLimits::LimitDC(double dc, units::radian_t position) {
  if (!using_limits_) return dc;

  if (dc < 0 && position <= reverse_limit_) {
    return 0.0;
  } else if (dc > 0 && position >= forward_limit_) {
    return 0.0;
  }

  if (position > forward_reduce_ && dc > reduce_max_dc_) {
    return reduce_max_dc_;
  } else if (position < reverse_reduce_ && dc < -reduce_max_dc_) {
    return -reduce_max_dc_;
  }

  return dc;
}

}  // namespace frc846::control::config