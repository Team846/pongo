#include "commands/teleop/drive_to_processor_command.h"

#include <iostream>

DriveToProcessorCommand::DriveToProcessorCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : DriveToPointCommand{drivetrain, {{0_in, 100_in}, 0_deg, 0_fps}, max_speed,
          max_acceleration, max_deceleration, true} {}

std::pair<frc846::math::FieldPoint, bool>
DriveToProcessorCommand::GetTargetPoint() {
  auto cpos = drivetrain_->GetReadings().estimated_pose.position;

  frc846::math::FieldPoint target_pos = {
      {295.75_in, 235.73_in}, -90_deg, 0_fps};
  target_pos = target_pos.mirror(
      frc::DriverStation::GetAlliance() ==
      frc::DriverStation::kBlue);  // TODO: make these points scriptable

  units::inch_t proc_drive_subtract =
      (drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "lock_drive_early"));
  target_pos.point =
      cpos + (target_pos.point - cpos).AddToMagnitude(-proc_drive_subtract);

  if ((target_pos.point - start_point_).magnitude() <= proc_drive_subtract) {
    target_pos.point = start_point_;
  }

  target_pos.velocity =
      drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
          "lock_drive_fvel");

  return {target_pos, true};
}