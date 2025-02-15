#include "commands/teleop/drive_to_reef_command.h"

#include <iostream>

#include "reef.h"

DriveToReefCommand::DriveToReefCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain, bool is_left,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : DriveToPointCommand{drivetrain,
          ReefProvider::getReefScoringLocations()[0], max_speed,
          max_acceleration, max_deceleration, true},
      is_left_{is_left} {}

std::pair<frc846::math::FieldPoint, bool> DriveToReefCommand::GetTargetPoint() {
  auto cpos = drivetrain_->GetReadings().estimated_pose.position;
  int reef_target_pos = ReefProvider::getClosestReefSide(cpos);
  Graph("reef_target_pos", reef_target_pos);
  auto target_pos =
      ReefProvider::getReefScoringLocations()[2 * reef_target_pos +
                                              (is_left_ ? 0 : 1)];

  units::inch_t reef_drive_subtract =
      (drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "lock_drive_early"));
  target_pos.point =
      cpos + (target_pos.point - cpos).AddToMagnitude(-reef_drive_subtract);

  if ((target_pos.point - start_point_).magnitude() <= reef_drive_subtract) {
    target_pos.point = start_point_;
  }

  target_pos.velocity =
      drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
          "lock_drive_fvel");

  return {target_pos, true};
}