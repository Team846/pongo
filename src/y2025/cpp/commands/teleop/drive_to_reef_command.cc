#include "commands/teleop/drive_to_reef_command.h"

#include "reef.h"

DriveToReefCommand::DriveToReefCommand(
    frc846::robot::swerve::DrivetrainSubsystem* drivetrain, bool is_left,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : DriveToPointCommand{drivetrain,
          ReefProvider::getReefScoringLocations()[0], max_speed,
          max_acceleration, max_deceleration},
      is_left_{is_left} {}

std::pair<frc846::math::FieldPoint, bool> DriveToReefCommand::GetTargetPoint() {
  auto cpos = drivetrain_->GetReadings().estimated_pose.position;
  int reef_target_pos = ReefProvider::getClosestReefSide(cpos);
  auto target_pos =
      ReefProvider::getReefScoringLocations()[2 * reef_target_pos +
                                              (is_left_ ? 0 : 1)];
  return {target_pos, true};
}