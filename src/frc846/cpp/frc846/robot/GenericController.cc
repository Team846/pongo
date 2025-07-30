#include "frc846/robot/GenericController.h"

namespace frc846::robot {

GenericControllerReadings::GenericControllerReadings(
    frc::GenericHID& controller)
    : one_button(controller.GetRawButton(1)),
      two_button(controller.GetRawButton(2)),
      three_button(controller.GetRawButton(3)),
      four_button(controller.GetRawButton(4)),
      five_button(controller.GetRawButton(5)),
      six_button(controller.GetRawButton(6)),
      seven_button(controller.GetRawButton(7)),
      eight_button(controller.GetRawButton(8)) {}

}  // namespace frc846::robot