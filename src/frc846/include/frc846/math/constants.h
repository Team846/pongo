#pragma once

#include <units/acceleration.h>

namespace frc846::math {

struct constants {
  struct physics {
    static constexpr units::feet_per_second_squared_t g{32.17405};
  };
  struct geometry {
    static constexpr double pi = 3.14159265;
  };
};

}  // namespace frc846::math