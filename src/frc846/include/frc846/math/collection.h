#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

namespace frc846::math {

// Double comparison using an epsilon value. Default epsilon value is 1e-9.
bool DEquals(double x, double y, double epsilon = 1e-9);

// Find the circumference of a circle given radius.
constexpr units::inch_t Circumference(units::meter_t radius) {
  return 2 * units::constants::pi * radius;
}

double HorizontalDeadband(double input, double x_intercept, double max,
                          double exponent = 1, double sensitivity = 1);

double VerticalDeadband(double input, double y_intercept, double max,
                        double exponent = 1, double sensitivity = 1);

// Returns the smallest difference between two angles
units::degree_t CoterminalDifference(units::degree_t angle,
                                     units::degree_t other_angle);

// Returns the smallest sum between two angles.
units::degree_t CoterminalSum(units::degree_t angle,
                              units::degree_t other_angle);

}  // namespace frc846::math