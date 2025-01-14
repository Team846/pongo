#pragma once

#include <units/length.h>

#include "frc846/math/vectors.h"

struct robot_constants {
  struct base {
    static constexpr units::inch_t wheelbase_x = 26_in;
    static constexpr units::inch_t wheelbase_y = 26_in;
    static constexpr units::pound_t weight = 75_lb;

    static constexpr units::inch_t height = 6_in;
  };

  struct elevator {
    static constexpr units::pound_t elevator_weight = 20_lb;
    static constexpr units::inch_t min_height_off_base = 38_in;

    static constexpr units::pound_t end_effector_weight = 10_lb;

    static constexpr units::pound_t total_weight =
        elevator_weight + end_effector_weight;

    static constexpr units::inch_t pos_x = -5_in;
    static constexpr units::inch_t pos_y = -7_in;
  };

  struct telescope {
    static constexpr units::pound_t total_weight = 20_lb;

    static constexpr units::inch_t pos_x = 5_in;
    static constexpr units::inch_t pos_y = -7_in;
  };

  static constexpr units::pound_t total_weight =
      base::weight + elevator::elevator_weight + elevator::end_effector_weight +
      telescope::total_weight;
};