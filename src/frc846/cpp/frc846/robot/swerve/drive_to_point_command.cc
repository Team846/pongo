#include "frc846/robot/swerve/drive_to_point_command.h"

namespace frc846::robot::swerve {

DriveToPointCommand::DriveToPointCommand(DrivetrainSubsystem* drivetrain,
    frc846::math::FieldPoint target, units::feet_per_second_t max_speed,
    units::feet_per_second_squared_t max_acceleration,
    units::feet_per_second_squared_t max_deceleration)
    : Loggable("DriveToPointCommand"),
      drivetrain_(drivetrain),
      max_speed_(max_speed),
      max_acceleration_(max_acceleration),
      max_deceleration_(max_deceleration),
      target_(target) {
  SetName("DriveToPointCommand");
  AddRequirements({drivetrain_});
}

void DriveToPointCommand::Initialize() {
  Log("DriveToPointCommand initialized");
  start_point_ = drivetrain_->GetReadings().pose.position;
  is_decelerating_ = false;
}

void DriveToPointCommand::Execute() {
  DrivetrainReadings dt_readings{drivetrain_->GetReadings()};

  DrivetrainAccelerationControlTarget dt_target{
      .linear_acceleration = max_acceleration_,
      .accel_dir = (target_.point - start_point_).angle(true),
      .angular_velocity = 0_deg_per_s,
  };

  units::second_t t_decel =
      ((dt_readings.pose.velocity.magnitude() - target_.velocity) /
          max_deceleration_);
  units::foot_t stopping_distance =
      units::math::abs(dt_readings.pose.velocity.magnitude() * t_decel) -
      ((max_deceleration_ * t_decel * t_decel) / 2.0);

  units::foot_t dist_to_target =
      (target_.point - dt_readings.pose.position).magnitude() -
      drivetrain_->GetPreferenceValue_unit_type<units::inch_t>(
          "drive_to_subtract");

  if (dist_to_target <= stopping_distance) {
    is_decelerating_ = true;
    dt_target.accel_dir = dt_readings.pose.velocity.angle(true) + 180_deg;

    if (dist_to_target > 3_in)
      dt_target.linear_acceleration =
          max_deceleration_ * dist_to_target / stopping_distance;
    else
      dt_target.linear_acceleration = max_deceleration_;
  } else {
    is_decelerating_ = false;
    if (dt_readings.pose.velocity.magnitude() < max_speed_)
      dt_target.linear_acceleration = max_acceleration_;
    else
      dt_target.linear_acceleration = 0_fps_sq;

    dt_target.accel_dir =
        (target_.point - dt_readings.pose.position).angle(true);
  }
  Graph("is_decelerating", is_decelerating_);

  dt_target.angular_velocity = drivetrain_->ApplyBearingPID(target_.bearing);

  drivetrain_->SetTarget(dt_target);
}

void DriveToPointCommand::End(bool interrupted) {
  Log("DriveToPointCommand ended with interruption status {}", interrupted);
  drivetrain_->SetTargetZero();
}

bool DriveToPointCommand::IsFinished() {
  auto drivetrain_readings = drivetrain_->GetReadings();
  auto current_point = drivetrain_readings.pose.position;
  return ((current_point - start_point_).magnitude() >=
             (target_.point - start_point_).magnitude()) ||
         ((drivetrain_readings.last_accel_spike <=
              drivetrain_->GetPreferenceValue_int("max_past_accel_spike")) &&
             drivetrain_readings.accel_vel <
                 drivetrain_
                     ->GetPreferenceValue_unit_type<units::feet_per_second_t>(
                         "accel_vel_bump_thresh"));
}

}  // namespace frc846::robot::swerve