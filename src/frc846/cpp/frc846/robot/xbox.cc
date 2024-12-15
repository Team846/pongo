#include "frc846/robot/xbox.h"

namespace frc846::robot {

XboxReadings::XboxReadings(frc::XboxController& xbox, double trigger_threshold)
    : left_stick_x(xbox.GetLeftX()),
      left_stick_y(-xbox.GetLeftY()),  // negated so + is up
      right_stick_x(xbox.GetRightX()),
      right_stick_y(-xbox.GetRightY()),  // negated so + is up
      left_trigger(xbox.GetLeftTriggerAxis() >= trigger_threshold),
      right_trigger(xbox.GetRightTriggerAxis() >= trigger_threshold),
      left_bumper(xbox.GetLeftBumper()),
      right_bumper(xbox.GetRightBumper()),
      back_button(xbox.GetBackButton()),
      start_button(xbox.GetStartButton()),
      rsb(xbox.GetRightStickButton()),
      a_button(xbox.GetAButton()),
      b_button(xbox.GetBButton()),
      x_button(xbox.GetXButton()),
      y_button(xbox.GetYButton()),
      pov(static_cast<frc846::robot::XboxPOV>(xbox.GetPOV())) {}

}  // namespace frc846::robot