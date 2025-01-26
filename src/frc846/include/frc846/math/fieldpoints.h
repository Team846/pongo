#pragma once

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>

#include "frc846/math/vectors.h"

namespace frc846::math {

struct FieldPoint {
  VectorND<units::foot, 2> point;
  units::degree_t bearing;

  VectorND<units::feet_per_second, 2> velocity;

  // Returns a FieldPoint that is 'mirrored' across the field
  FieldPoint mirror(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{{field_size_x - point[0], field_size_y - point[1]},
          180_deg + bearing, {velocity[0], velocity[1]}};
    }
    return FieldPoint{
        {point[0], point[1]}, bearing, {velocity[0], velocity[1]}};
  }

  // Returns a FieldPoint that is 'mirrored' across the centerline of the field
  FieldPoint mirrorOnlyY(bool shouldMirror = true) const {
    if (shouldMirror) {
      return FieldPoint{{point[0], field_size_y - point[1]}, 180_deg - bearing,
          {velocity[0], velocity[1]}};
    }
    return mirror(false);
  }

  static constexpr units::inch_t field_size_y = 651.25_in;
  static constexpr units::inch_t field_size_x = 315.5_in;
};

}  // namespace frc846::math