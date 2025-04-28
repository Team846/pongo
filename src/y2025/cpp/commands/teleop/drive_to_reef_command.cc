#include "commands/teleop/drive_to_reef_command.h"

#include <iostream>

#include "reef.h"

DriveToReefCommand::DriveToReefCommand(RobotContainer& container, bool is_left,
    bool is_pre_point, units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : DriveToPointCommand{&container.drivetrain_,
          ReefProvider::getReefScoringLocations()[0], max_speed,
          max_acceleration, max_deceleration, true},
      container_{container},
      is_left_{is_left},
      is_pre_point_{is_pre_point} {}

std::pair<frc846::math::FieldPoint, bool> DriveToReefCommand::GetTargetPoint() {
  auto cpos = drivetrain_->GetReadings().estimated_pose.position;
  auto cvel = drivetrain_->GetReadings().estimated_pose.velocity;
  bool use_pred_pos = false;
  frc846::math::Vector2D predicted_pos;

  if (cvel.magnitude() > 2_fps) {
    use_pred_pos = true;
    units::second_t time_step = 0.3_s;
    predicted_pos =
        cpos + frc846::math::Vector2D{cvel[0] * time_step, cvel[1] * time_step};
  }
  int reef_target_pos =
      ReefProvider::getClosestReefSide(use_pred_pos ? predicted_pos : cpos);
  Graph("reef_target_pos", reef_target_pos);
  auto target_pos = ReefProvider::getReefScoringLocations(true, is_pre_point_,
      !(container_.control_input_.GetReadings().coral_state == kCoral_ScoreL2 ||
          container_.control_input_.GetReadings().coral_state ==
              kCoral_ScoreL3))[2 * reef_target_pos + (is_left_ ? 0 : 1)];

  //   units::inch_t reef_drive_subtract =
  //       (drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
  //           "lock_drive_early"));
  //   target_pos.point =
  //       cpos + (target_pos.point -
  //       cpos).AddToMagnitude(-reef_drive_subtract);

  //   if ((target_pos.point - start_point_).magnitude() <= reef_drive_subtract)
  //   {
  //     target_pos.point = start_point_;
  //   }

  //   target_pos.velocity =
  //       drivetrain_->GetPreferenceValue_unit_type<units::feet_per_second_t>(
  //           "lock_drive_fvel");

  // if (is_pre_point_) { target_pos.velocity = 2_fps; }

  return {target_pos, true};
}