#pragma once

#include <frc/XboxController.h>

namespace frc846::robot {
enum class XboxPOV : int {
  kNone = -1,
  kUp = 0,
  kUpRight = 45,
  kRight = 90,
  kDownRight = 135,
  kDown = 180,
  kDownLeft = 225,
  kLeft = 270,
  kUpLeft = 315
};

struct XboxReadings {
  double left_stick_x;   // [-1, 1]
  double left_stick_y;   // [-1, 1]
  double right_stick_x;  // [-1, 1]
  double right_stick_y;  // [-1, 1]

  bool left_trigger;
  bool right_trigger;
  bool left_bumper;
  bool right_bumper;

  bool back_button;
  bool start_button;
  bool lsb;
  bool rsb;

  bool a_button;
  bool b_button;
  bool x_button;
  bool y_button;

  frc846::robot::XboxPOV pov;

  XboxReadings() = default;
  XboxReadings(frc::XboxController& xbox, double trigger_threshold);
};

}  // namespace frc846::robot
