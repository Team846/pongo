#pragma once

#include <frc/GenericHID.h>

namespace frc846::robot {

struct GenericControllerReadings {
  bool one_button;
  bool two_button;
  bool three_button;
  bool four_button;
  bool five_button;
  bool six_button;
  bool seven_button;
  bool eight_button;

  GenericControllerReadings() = default;
  GenericControllerReadings(frc::GenericHID& controller);
};

}  // namespace frc846::robot