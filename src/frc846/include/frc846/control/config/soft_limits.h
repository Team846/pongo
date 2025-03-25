#pragma once

#include <units/angular_velocity.h>

#include <cassert>

namespace frc846::control::config {

/*
SoftLimits

A non-templated class that implements soft limits. Meant for use by
HigherMotorController.
*/
class SoftLimits {
public:
  SoftLimits(bool using_limits, units::radian_t forward_limit,
      units::radian_t reverse_limit, units::radian_t forward_reduce,
      units::radian_t reverse_reduce, double reduce_max_dc);

  units::radian_t LimitPosition(units::radian_t position);

  units::radians_per_second_t LimitVelocity(
      units::radians_per_second_t velocity, units::radian_t position);

  double LimitDC(double dc, units::radian_t position);

  bool using_limits_;

private:
  units::radian_t forward_limit_;
  units::radian_t reverse_limit_;
  units::radian_t forward_reduce_;
  units::radian_t reverse_reduce_;
  double reduce_max_dc_;
};

/*
SoftLimitsHelper

A templated class to help create SoftLimits objects from system units.
*/
template <typename T> class SoftLimitsHelper {
public:
  using pos_unit = units::unit_t<T>;
  using conv_unit =
      units::unit_t<units::compound_unit<T, units::inverse<units::turn>>>;

  static SoftLimits CreateSoftLimits(conv_unit conversion, bool using_limits,
      pos_unit forward_limit, pos_unit reverse_limit, pos_unit forward_reduce,
      pos_unit reverse_reduce, double reduce_max_dc) {
    return SoftLimits{using_limits, forward_limit / conversion,
        reverse_limit / conversion, forward_reduce / conversion,
        reverse_reduce / conversion, reduce_max_dc};
  }
};

}  // namespace frc846::control::config