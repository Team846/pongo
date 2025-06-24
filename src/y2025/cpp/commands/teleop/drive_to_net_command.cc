#include "commands/teleop/drive_to_net_command.h"

#include <iostream>

DriveToNetCommand::DriveToNetCommand(RobotContainer& container, bool is_scoring,
    units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : frc846::robot::swerve::DriveToPointCommand{&container.drivetrain_,
          frc846::math::FieldPoint{{0_in, 0_in}, 0_deg, 0_fps}, max_speed,
          max_acceleration, max_deceleration, true},
      container_{container},
      is_scoring_{is_scoring} {}

std::pair<frc846::math::FieldPoint, bool> DriveToNetCommand::GetTargetPoint() {
  frc846::math::Vector2D pos =
      drivetrain_->GetReadings().estimated_pose.position;
  units::degree_t bearing = drivetrain_->GetReadings().estimated_pose.bearing;

  units::inch_t net_offest = 55_in;
  if (is_scoring_) { net_offest = 45_in; }
  frc846::math::Vector2D net_pos = frc846::math::Vector2D{
      pos[0], frc846::math::FieldPoint::field_size_y / 2 - net_offest};
  frc846::math::FieldPoint target_pos =
      frc846::math::FieldPoint{net_pos, 0_deg, 0_fps};
  if (bearing > 150_deg) { target_pos = target_pos.mirrorOnlyY(true); }

  return {target_pos, true};
}