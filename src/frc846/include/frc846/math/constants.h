#pragma once

#include <units/acceleration.h>

namespace frc846::math {

struct constants {
  struct physics {
    static constexpr units::feet_per_second_squared_t g{32.17405};
  };
  static constexpr double pi = 3.14159265358979323846;
  static constexpr double e = 2.71828182845904523536;
};

}  // namespace frc846::math