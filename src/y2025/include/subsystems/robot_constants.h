#pragma once

#include <units/length.h>

#include "frc846/control/calculators/CircuitResistanceCalculator.h"
#include "frc846/math/vectors.h"

struct robot_constants {
  struct base {
    static constexpr units::inch_t wheelbase_x = 22.5_in;
    static constexpr units::inch_t wheelbase_y = 25.5_in;
    static constexpr units::pound_t weight = 60_lb;

    static constexpr units::inch_t height = 1.5_in;
  };

  static constexpr units::pound_t total_weight =
      base::weight;
};