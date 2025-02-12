#include "subsystems/hardware/leds_logic.h"

void LEDsLogic::UpdateLEDs(RobotContainer* container) {
  LEDsTarget target{kLEDsUnready};
  if (frc::DriverStation::IsDisabled() &&
      (!container->coral_ss_.isHomed() || !container->algal_ss_.isHomed())) {
    target.state = kLEDsUnready;
  } else if (frc::DriverStation::IsDisabled()) {
    target.state = kLEDsDisabled;
  } else if (frc::DriverStation::IsAutonomous() ||
             frc::DriverStation::IsAutonomousEnabled()) {
    target.state = kLEDsAutonomous;
  } else if (container->control_input_.GetReadings().auto_align ||
             container->control_input_.GetReadings().lock_left_reef ||
             container->control_input_.GetReadings().lock_right_reef) {
    // TODO: add GPD and other sequencing to LEDs as well
    target.state = kLEDsSequencing;
  } else if (container->control_input_.GetReadings().climb_state != 0) {
    target.state = kLEDsClimbing;
  } else if (container->algal_ss_.algal_end_effector.GetReadings().has_piece_ ||
             container->coral_ss_.coral_end_effector.GetReadings().has_piece_) {
    target.state = kLEDsHavePiece;
  } else if (frc::DriverStation::IsTeleop()) {
    target.state = kLEDsTeleop;
  }

  container->leds_.SetTarget(target);
}