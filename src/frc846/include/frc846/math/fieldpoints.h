#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>

#include "frc846/math/vectors.h"

namespace frc846::math {

struct FieldPoint {
  Vector2D point;
  units::degree_t bearing;

  units::feet_per_second_t velocity;

  // Returns a FieldPoint that is 'mirrored' across the field
  FieldPoint mirror(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{{field_size_x - point[0], field_size_y - point[1]},
          180_deg + bearing, velocity};
    }
    return FieldPoint{{point[0], point[1]}, bearing, velocity};
  }

  // Returns a FieldPoint that is mirrored across the centerline of the field
  FieldPoint mirrorOnlyY(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{
          {point[0], field_size_y - point[1]}, 180_deg - bearing, velocity};
    }
    return mirror(false);
  }

  // Returns a FieldPoint that is mirrored along the length-axis of the field
  FieldPoint mirrorOnlyX(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{
          {field_size_x - point[0], point[1]}, -bearing, velocity};
    }
    return mirror(false);
  }

private:
  static constexpr units::inch_t field_size_y = 690.875_in;
  static constexpr units::inch_t field_size_x = 317_in;
};

}  // namespace frc846::math