#include "subsystems/hardware/leds_logic.h"

#include <iostream>

#include "frc846/math/fieldpoints.h"
#include "reef.h"

LEDsState LEDsLogic::checkLoc(RobotContainer* container, LEDsTarget target) {
  std::string selected_auto = frc846::robot::GenericRobot::GetSelectedAuto();

  units::inch_t auto_y = (290_in);

  units::inch_t auto_3pc = (158.5_in - 73.25_in);
  units::inch_t auto_1pc = (158.5_in);
  units::inch_t auto_leave = (158.5_in);

  try {
    if (selected_auto.substr(0, 3) == "5PC") {
      std::string blue_red = selected_auto.substr(4, 5);
      std::string left_right = selected_auto.substr(6);

      frc846::math::FieldPoint auto_start = {{auto_3pc, auto_y}, 0_deg, 0_fps};

      if (blue_red == "B") { auto_start = auto_start.mirror(true); }
      if (left_right == "R") { auto_start = auto_start.mirrorOnlyX(true); }

      if ((auto_start.point -
              container->drivetrain_.GetReadings().estimated_pose.position)
              .magnitude() < 1_ft) {
        return kisLinedUp;
      }
    }
    if (selected_auto.substr(0, 4) == "1PCN") {
      std::string blue_red = selected_auto.substr(5, 6);
      std::string left_right = selected_auto.substr(7);

      frc846::math::FieldPoint auto_start = {{auto_1pc, auto_y}, 0_deg, 0_fps};

      if (blue_red == "B") { auto_start = auto_start.mirror(true); }
      if (left_right == "R") { auto_start = auto_start.mirrorOnlyX(true); }

      if ((auto_start.point -
              container->drivetrain_.GetReadings().estimated_pose.position)
              .magnitude() < 1_ft) {
        return kisLinedUp;
      }
    }
    if (selected_auto.substr(0, 5) == "LEAVE") {
      std::string blue_red = selected_auto.substr(6, 7);
      std::string left_right = selected_auto.substr(8);

      frc846::math::FieldPoint auto_start = {
          {auto_leave, auto_y}, 0_deg, 0_fps};

      if (blue_red == "B") { auto_start = auto_start.mirror(true); }
      if (left_right == "R") { auto_start = auto_start.mirrorOnlyX(true); }

      if ((auto_start.point -
              container->drivetrain_.GetReadings().estimated_pose.position)
              .magnitude() < 1_ft) {
        return kisLinedUp;
      }
    }
  } catch (const std::exception& exc) { (void)exc; }

  return kLEDsDisabled;
}

bool LEDsLogic::reachedAutoAlignTarget(RobotContainer* container) {
  auto reefTargetPos =
      ReefProvider::getReefScoringLocations()[ReefProvider::getClosestReefSide(
          container->drivetrain_.GetReadings().estimated_pose.position)];

  if ((reefTargetPos.point -
          container->drivetrain_.GetReadings().estimated_pose.position)
          .magnitude() < 1.5_in) {
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
             (container->control_input_.GetReadings().targeting_algae &&
                 container->GPD_.GetReadings().gamepieces.size() != 0U)) {
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