#include "subsystems/hardware/leds_logic.h"

#include <iostream>

#include "frc846/math/fieldpoints.h"
#include "reef.h"

LEDsState LEDsLogic::checkLoc(RobotContainer* container, LEDsTarget target) {
  std::string selected_auto = frc846::robot::GenericRobot::GetSelectedAuto();

  frc846::math::FieldPoint auto_start = {{36.5_in, 265.3_in}, 120_deg, 0_fps};

  try {
    if (selected_auto.substr(0, 3) == "5PC") {
      std::string blue_red = selected_auto.substr(4, 5);
      std::string left_right = selected_auto.substr(6);

      if (blue_red == "B") auto_start = auto_start.mirror(true);
      if (left_right == "R") auto_start = auto_start.mirrorOnlyX(true);

      if ((auto_start.point -
              container->drivetrain_.GetReadings().estimated_pose.position)
              .magnitude() < 1_ft) {
        if (units::math::abs(
                container->drivetrain_.GetReadings().estimated_pose.bearing -
                auto_start.bearing) < 10_deg) {
          return kisCompletelyLinedUp;
        }
        return kisLinedUp;
      }
    }
  } catch (const std::exception& exc) { (void)exc; }

  return kLEDsDisabled;
}

bool LEDsLogic::reachedAutoAlignTarget(RobotContainer* container) {
  bool l4 = !(
      container->control_input_.GetReadings().coral_state == kCoral_ScoreL3 ||
      container->control_input_.GetReadings().coral_state == kCoral_ScoreL2);
  auto reefTargetPos = ReefProvider::getReefScoringLocations(
      true, false, l4)[ReefProvider::getClosestReefSide(
      container->drivetrain_.GetReadings().estimated_pose.position)];

  if ((reefTargetPos.point -
          container->drivetrain_.GetReadings().estimated_pose.position)
          .magnitude() < 0.5_in) {
    return true;
  }
  return false;
}

void LEDsLogic::UpdateLEDs(RobotContainer* container) {
  LEDsTarget target{kLEDsUnready};
  if (frc::DriverStation::IsDisabled() &&
      (!container->coral_ss_.isHomed() || !container->algal_ss_.isHomed())) {
    target.state = kLEDsUnready;
  } else if (frc::DriverStation::IsDisabled()) {
    target.state = checkLoc(container, target);
  } else if (frc::DriverStation::IsAutonomous() ||
             frc::DriverStation::IsAutonomousEnabled()) {
    target.state = kLEDsAutonomous;
  } else if (container->control_input_.GetReadings().lock_left_reef ||
             container->control_input_.GetReadings().lock_right_reef ||
             container->control_input_.GetReadings().lock_net ||
             container->control_input_.GetReadings().auto_align) {
    if (reachedAutoAlignTarget(container))
      target.state = kisLinedUp;
    else
      target.state = kLEDsSequencing;
  } else if (container->control_input_.GetReadings().extend_climb ||
             container->control_input_.GetReadings().retract_climb) {
    target.state = kLEDsClimbing;
  } else if (container->algal_ss_.algal_end_effector.GetReadings().has_piece_ ||
             container->coral_ss_.coral_end_effector.GetReadings().has_piece_ ||
             frc::DriverStation::IsTest()) {
    target.state = kLEDsHavePiece;
  } else if (frc::DriverStation::IsTeleop()) {
    target.state = kLEDsTeleop;
  }

  container->leds_.SetTarget(target);
}

void LEDsLogic::SetLEDsState(RobotContainer* container, LEDsState state) {
  LEDsTarget target{state};
  container->leds_.SetTarget(target);
}

void LEDsLogic::CoastingLEDs(RobotContainer* container, double percent) {
  container->leds_.SetTarget(LEDsCoastingTarget{percent});
}