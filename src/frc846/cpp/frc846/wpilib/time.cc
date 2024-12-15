#include "frc846/wpilib/time.h"

namespace frc846::wpilib {

units::second_t CurrentFPGATime() {
  int err;
  return units::microsecond_t(HAL_GetFPGATime(&err));
}

}  // namespace frc846::wpilib