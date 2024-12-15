#include "frc846/math/collection.h"

#include <cmath>

namespace frc846::math {

bool DEquals(double x, double y, double epsilon) {
  return std::abs(x - y) < epsilon;
}

double HorizontalDeadband(double input, double x_intercept, double max,
                          double exponent, double sensitivity) {
  double y = 0;

  auto slope = max / (max - x_intercept);
  if (input > x_intercept) {
    y = (input - x_intercept) * slope;
  } else if (input < -x_intercept) {
    y = (input + x_intercept) * slope;
  }

  return copysign(max * pow(y / max, exponent) * sensitivity, input);
}

double VerticalDeadband(double input, double y_intercept, double max,
                        double exponent, double sensitivity) {
  double y = 0;

  auto slope = (max - y_intercept) / max;
  if (input > 0) {
    y = input * slope + y_intercept;
  } else if (input < 0) {
    y = input * slope - y_intercept;
  }

  return copysign(max * pow(y / max, exponent) * sensitivity, input);
}

units::degree_t CoterminalDifference(units::degree_t angle,
                                     units::degree_t other_angle) {
  const units::angle::degree_t difference =
      units::math::fmod(angle, 1_tr) - units::math::fmod(other_angle, 1_tr);
  if (difference > 0.5_tr) {
    return difference - 1_tr;
  } else if (difference < -0.5_tr) {
    return difference + 1_tr;
  } else {
    return difference;
  }
}

units::degree_t CoterminalSum(units::degree_t angle,
                              units::degree_t other_angle) {
  const units::angle::degree_t difference =
      units::math::fmod(angle, 1_tr) + units::math::fmod(other_angle, 1_tr);
  if (difference > 0.5_tr) {
    return difference - 1_tr;
  } else if (difference < -0.5_tr) {
    return difference + 1_tr;
  } else {
    return difference;
  }
}

}  // namespace frc846::math